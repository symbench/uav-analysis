#!/usr/bin/env python3
# Copyright (C) 2022, Miklos Maroti
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from typing import Any, Dict, Generator

import csv
import math
import os
import sys

from .components import MOTORS, PROPELLERS

DATAPATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                        'data_demo1')


def csv_generator(filename: str) -> Generator[Dict[str, Any], None, None]:
    with open(os.path.join(DATAPATH, filename)) as file:
        reader = csv.DictReader(file)
        for line in reader:
            yield dict(line)


def flatten(data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict()
    for key, val in data.items():
        if isinstance(val, dict):
            key += "."
            for key2, val2 in val.items():
                result[key + key2] = val2
        else:
            result[key] = val
    return result


def battery_generator(battery_bug: bool) -> \
        Generator[Dict[str, Any], None, None]:
    for data in csv_generator("Battery.csv"):
        discharge_rate = float(data["Cont. Discharge Rate [C]"])
        capacity = float(data["Capacity [Ah]"])
        if battery_bug:
            discharge_rate = 25.0

        yield {
            "weight": float(data["Weight [kg]"]),           # kg
            "volume": float(data["Volume [mm^3]"]) * 1e-9,  # m^3
            "energy": float(data["Power [Wh]"]),            # Wh
            "voltage": float(data["Min Voltage [V]"]),      # V
            "current": capacity * discharge_rate,           # A
            "name": data["Name"],
        }


def battery_pack_generator(
        max_voltage: float,
        min_current: float,
        min_energy: float,
        battery_bug: bool) -> Generator[Dict[str, Any], None, None]:
    """
    Iterate through the battery packs that can generate voltage between 
    90% and 100% of the specified max_voltage, can provide at least 
    min_current current continuously, and have at least the given 
    min_energy energy.
    """

    for data in battery_generator(battery_bug):
        series_count = int(math.floor(
            max_voltage / data["voltage"]))

        battery_max_voltage = data["voltage"] * series_count
        if battery_max_voltage < max_voltage * 0.9:
            continue

        parallel_count1 = int(math.ceil(
            min_current / data["current"]))
        parallel_count2 = int(math.ceil(
            min_energy / (data["energy"] * series_count)))
        parallel_count = max(parallel_count1, parallel_count2)

        yield {
            "weight": data["weight"] * series_count * parallel_count,
            "volume": data["volume"] * series_count * parallel_count,
            "energy": data["energy"] * series_count * parallel_count,
            "voltage": battery_max_voltage,
            "current": data["current"] * parallel_count,
            "series_count": series_count,
            "parallel_count": parallel_count,
            "name": data["name"],
        }


def wing_generator() -> Generator[Dict[str, Any], None, None]:
    for data in csv_generator("wing_analysis_pareto.csv"):
        assert float(data["speed"]) == 50.0
        yield {
            "weight": float(data["weight"]),       # kg
            "lift_50mps": float(data["lift"]),     # N
            "drag_50mps": float(data["drag"]),     # N
            "available_volume": float(data["available_volume"]) * 1e-9,  # m^3
            "profile_name": data["profile"],
            "chord": float(data["chord"]) * 1e-3,  # m
            "span": float(data["span"]) * 1e-3,    # m
        }


def motor_propeller_generator() -> \
        Generator[Dict[str, Any], None, None]:
    for data in csv_generator("motor_propeller_analysis_pos_thrust_neg_power_weight.csv"):
        yield {
            "weight": float(data["weight"]),            # kg
            "thrust": float(data["thrust"]),            # N
            "power": float(data["power"]),              # W
            "max_voltage": float(data["max_voltage"]),  # V
            "min_current": float(data["max_current"]),  # A
            "motor_name": data["motor_name"],
            "propeller_name": data["propeller_name"],
        }


def multi_motor_propeller_generator(
        min_count: int,
        max_count: int) -> Generator[Dict[str, Any], None, None]:
    assert min_count <= max_count
    for data in motor_propeller_generator():
        for count in range(min_count, max_count + 1):
            yield flatten({
                "weight": data["weight"] * count,            # kg
                "thrust": data["thrust"] * count,            # N
                "power": data["power"] * count,              # W
                "max_voltage": data["max_voltage"],          # V
                "min_current": data["min_current"] * count,  # A
                "count": count,
                "motor_propeller": data,
            })


def battery_double_motor_propeller_generator(
        forward_count: int,
        lifting_count: int,
        min_seconds: float,
        max_weight: float,
        battery_bug: bool) -> Generator[Dict[str, Any], None, None]:
    min_hours = min_seconds / 3600.0
    for forward in multi_motor_propeller_generator(
            min_count=forward_count,
            max_count=forward_count):
        for lifting in multi_motor_propeller_generator(
                min_count=lifting_count,
                max_count=lifting_count):

            # it is not clear if we need this or nto
            ratio = max(forward["max_voltage"], lifting["max_voltage"]) / \
                min(forward["max_voltage"], lifting["max_voltage"])
            if ratio > 1.1:
                continue

            max_voltage = min(forward["max_voltage"], lifting["max_voltage"])
            min_current = max(forward["min_current"], lifting["min_current"])
            min_energy = (forward["power"] + lifting["power"]) * min_hours

            for battery_pack in battery_pack_generator(
                    max_voltage=max_voltage,
                    min_current=min_current,
                    min_energy=min_energy,
                    battery_bug=battery_bug):

                weight = forward["weight"] + \
                    lifting["weight"] + battery_pack["weight"]
                if weight > max_weight:
                    continue

                yield flatten({
                    "forward_thrust": forward["thrust"],
                    "lifting_thrust": lifting["thrust"],
                    "weight": weight,
                    "forward": forward,
                    "lifting": lifting,
                    "battery_pack": battery_pack,
                })


def wing_battery_num_motors_generator(
        battery_bug: True,
        min_forward: int = 10, max_forward: int = 20,
        min_lifting: int = 10, max_lifting: int = 20):
    import json

    # TODO: this is not a good approach probably, will do it with constraint solver

    # hand picked
    motor_prop = {
        "motor_name": "KDE13218XF-105",
        "propeller_name": "34x3_2_4600_41_250",
        "weight": 2.313,
        "propeller_diameter": 0.8636,
        "voltage": 25.0,
        "thrust": 106.04,
        "power": 1743.54,
        "current": 69.74,
        "max_voltage": 38.29,
    }

    num_wings = 2

    for batt in battery_generator(battery_bug=battery_bug):
        series_count = int(math.ceil(
            motor_prop["voltage"] / batt["voltage"]))

        pack_voltage = batt["voltage"] * series_count
        if pack_voltage > motor_prop["max_voltage"]:
            continue

        min_parallel_count = int(math.ceil(
            motor_prop["current"] / batt["current"]))

        for wing in wing_generator():
            wing_weight = wing["weight"] * num_wings

            max_parallal_count = int(math.floor(
                (num_wings * wing["available_volume"]) /
                (batt["volume"] * series_count)))

            if max_parallal_count < min_parallel_count:
                continue

            print(json.dumps({
                "motor_prop": motor_prop,
                "battery": batt,
                "wing": wing,
            }, indent=2))


def save_to_csv(generator: Generator[Dict[str, Any], None, None],
                filename: str):
    count = 0
    with open(filename, 'w', newline='') as file:
        data = next(generator)
        count += 1
        writer = csv.DictWriter(file, data.keys())
        writer.writeheader()
        writer.writerow(data)
        for data in generator:
            writer.writerow(data)
            count += 1
            if count % 100 == 0:
                sys.stdout.write("\r{} designs".format(count))
                sys.stdout.flush()

    print("\nSaved {} designs to {}".format(count, filename))


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output', default='aggregate_analysis.csv',
                        help="output file name")
    parser.add_argument('--forward', type=int, metavar='NUM', default=2,
                        help='number of forward propellers')
    parser.add_argument('--lifting', type=int, metavar='NUM', default=8,
                        help='number of lifting propellers')
    parser.add_argument('--seconds', type=float, metavar='SEC', default=300.0,
                        help='maximum duration in seconds')
    parser.add_argument('--max-weight', type=float, metavar='KG', default=1000.0,
                        help='maximum weight of propellers, motors and batteries')
    parser.add_argument('--battery-bug', action='store_true', default=False,
                        help='sets the discharge rate to 25C')
    parser.add_argument('generator', choices=[
        "double-motor-propeller",
        "wing-battery-num-motors",
    ])

    args = parser.parse_args(args)

    if args.generator == "double-motor-propeller":
        generator = battery_double_motor_propeller_generator(
            forward_count=args.forward,
            lifting_count=args.lifting,
            min_seconds=args.seconds,
            max_weight=args.max_weight,
            battery_bug=args.battery_bug)
    elif args.generator == "wing-battery-num-motors":
        generator = wing_battery_num_motors_generator(
            battery_bug=args.battery_bug,
            min_forward=args.forward,
            max_forward=args.forward,
            min_lifting=args.lifting,
            max_lifting=args.lifting)
    else:
        raise ValueError("unknown generator")

    # save_to_csv(generator, args.output)


if __name__ == '__main__':
    run()
