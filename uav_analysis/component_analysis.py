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
import os

DATAPATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                        'data_hackathon2_uam')


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


def wing_generator() -> \
        Generator[Dict[str, Any], None, None]:
    for data in csv_generator("wing_analysis_pareto.csv"):
        assert float(data["speed"]) == 50.0
        yield {
            "weight": float(data["weight"]),      # kg
            "lift_50mps": float(data["lift"]),    # N
            "drag_50mps": float(data["drag"]),    # N
            "available_volume": float(data["available_volume"]),  # m
            "profile_name": data["profile"],
            "chord": float(data["chord"]),
            "span": float(data["span"]),
        }


def motor_propeller_generator() -> \
        Generator[Dict[str, Any], None, None]:
    for data in csv_generator("motor_propeller_analysis_pareto.csv"):
        yield {
            "weight": float(data["Combined.Weight"]),   # kg
            "thrust_max": float(data["Combined.MaxThrust"]),  # N
            "voltage_max": float(data["Combined.MaxVoltage"]),  # V
            "current_max": float(data["Combined.MaxCurrent"]),  # A
            "power_max": float(data["Combined.MaxPower"]),  # W
            "motor_name": data["Motor.Name"],
            "propeller_name": data["Propeller.Name"],
        }


def multi_motor_propeller_generator(min_count: int, max_count: int) -> \
        Generator[Dict[str, Any], None, None]:
    assert min_count <= max_count
    for data in motor_propeller_generator():
        for count in range(min_count, max_count + 1):
            yield flatten({
                "weight": data["weight"] * count,
                "thrust_max": data["thrust_max"] * count,
                "voltage_max": data["voltage_max"],
                "current_max": data["current_max"] * count,
                "power_max": data["power_max"] * count,
                "motor_propeller_count": count,
                "motor_propeller": data,
            })


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    args = parser.parse_args(args)

    for data in multi_motor_propeller_generator(1, 20):
        print(data)


if __name__ == '__main__':
    run()
