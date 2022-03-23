#!/usr/bin/env python3
# Copyright (C) 2022, Kalmar Gyorgy and Miklos Maroti
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

import math
import sys
from typing import List, Iterable, Generator

from .components import BATTERIES


def combination_generator(
        batteries: Iterable[str],
        max_series: int,
        max_parallel: int
):
    for b in batteries:
        for s in range(1, max_series + 1):
            for p in range(1, max_parallel + 1):
                yield {
                    "battery": b,
                    "series_count": s,
                    "parallel_count": p,
                }


def generate_csv(
        fname: str,
        use_bug: bool,
        max_parallel: int,
        max_series: int,
        min_voltage: float,
        max_voltage: float,
        min_energy: float,
        max_weight: float,
        min_current: float):
    try:
        output_file = open(fname, 'w')
    except OSError:
        print("Could not open file:", fname)
        sys.exit()
    output_file.write(
        "NAME,BATTERY_VOLUME,BATTERY_WEIGHT,BATTERY_CAPACITY,BATTERY_CONT_DISCHARGE_RATE,SERIES_COUNT,PAR_COUNT,TOTAL_VOLUME,TOTAL_WEIGHT,TOTAL_ENERGY,TOTAL_VOLTAGE,TOTAL_CAPACITY,TOTAL_MAX_CURRENT\n")

    comb_generator = combination_generator(
        BATTERIES,
        max_series=max_series,
        max_parallel=max_parallel)

    count = 0
    for c in comb_generator:
        parallel_count = c["parallel_count"]
        series_count = c["series_count"]

        # print("Processing combination: ", c)
        battery = BATTERIES[c["battery"]]

        battery_volume = float(battery["Volume [mm^3]"]) * 1e-9  # in m^3
        battery_weight = float(battery["Weight [kg]"])

        if use_bug:
            battery_capacity = 25.0  # check run_fd_calc.py
            battery_cont_discharge_rate = 25.0
        else:
            battery_capacity = float(battery["Capacity [Ah]"])
            battery_cont_discharge_rate = float(
                battery["Cont. Discharge Rate [C]"])

        battery_voltage = float(battery["Min Voltage [V]"])
        battery_power = battery_capacity * battery_voltage

        total_volume = parallel_count * series_count * battery_volume

        total_weight = parallel_count * series_count * battery_weight
        if total_weight > max_weight:
            continue

        total_energy = parallel_count * series_count * battery_power
        if total_energy < min_energy:
            continue

        total_voltage = series_count * battery_voltage
        if total_voltage < min_voltage or total_voltage > max_voltage:
            continue

        total_capacity = parallel_count * battery_capacity
        total_max_current = total_capacity * battery_cont_discharge_rate
        if total_max_current < min_current:
            continue

        row_string = "{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            c["battery"],
            battery_volume,
            battery_weight,
            battery_capacity,
            battery_cont_discharge_rate,
            series_count,
            parallel_count,
            total_volume,       # (min)
            total_weight,       # min
            total_energy,       # max
            total_voltage,
            total_capacity,
            total_max_current,  # max
        )
        output_file.write(row_string)
        count += 1

    output_file.close()
    print("Result table with", count, "rows is saved to:", fname)


def battery_analyzer(batt_name="Tattu 22Ah Li", ):
    capacity = float(BATTERIES[batt_name]["Capacity [Ah]"])
    voltage_request = 51.8  # input
    base_voltage = float(BATTERIES[batt_name]["Min Voltage [V]"])
    module_volume = float(BATTERIES[batt_name]["Volume [mm^3]"])
    volume_percent = 10.0
    chord_1 = 1000.0  # does not match wings csv
    chord_2 = 1000.0  # does not match wings csv
    thickness = 12.0  # where does it come from?
    span = 3000.0  # does not match wings csv

    series_count = math.ceil(voltage_request / base_voltage)
    min_pack_volume = series_count * module_volume
    volume_ratio = volume_percent / 100

    c_continuous = float(BATTERIES[batt_name]
                         ["Cont. Discharge Rate [C]"])  # not used
    c_peak = float(BATTERIES[batt_name]["Peak Discharge Rate [C]"])  # not used

    root_chord = max([chord_1, chord_2])
    tip_chord = min([chord_1, chord_2])
    A = root_chord / 2
    B = thickness / 100 * A
    C = tip_chord / 2
    D = thickness / 100 * C

    slope = (C - A) / span  # not used
    available_volume = 1 / 6 * span * (A * B + C * D + ((A + C) * (B + D)))
    goal_volume = available_volume * volume_ratio

    p_count = math.floor(goal_volume / min_pack_volume)
    mega_pack_capacity = capacity * p_count  # output
    actual_voltage = base_voltage * series_count  # output
    print("Direct solution: ", end='')
    print(mega_pack_capacity)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser()
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('-a', '--a-batt', nargs='?', metavar="STR",
                        const="Tattu 22Ah Li", help="Analyzes given battery")
    action.add_argument('-o', '--output', nargs='?', metavar="PATH",
                        const="battery_analysis.csv",
                        help="Generates battery analysis table")
    parser.add_argument('--use-bug', action="store_true", default=False,
                        help="Ignores actual battery capacity and discharge rate")
    parser.add_argument('--max-parallel', type=int, default=500, metavar="NUM",
                        help="sets the maximum parallel count")
    parser.add_argument('--max-series', type=int, default=100, metavar="NUM",
                        help="sets the maximum series count")
    parser.add_argument('--min-voltage', type=float, default=100, metavar="V",
                        help="sets the minimum total voltage")
    parser.add_argument('--max-voltage', type=float, default=1000, metavar="V",
                        help="sets the maximum total voltage")
    parser.add_argument('--min-energy', type=float, default=1000, metavar="WH",
                        help="sets the minimum total energy in What hour")
    parser.add_argument('--max-weight', type=float, default=500, metavar="KG",
                        help="sets the maximum total weight")
    parser.add_argument('--min-current', type=float, default=10, metavar="A",
                        help="sets the minimum max current in Ampere")

    args = parser.parse_args(args)
    if args.output is not None:
        generate_csv(
            fname=args.output,
            use_bug=args.use_bug,
            max_parallel=args.max_parallel,
            max_series=args.max_series,
            min_voltage=args.min_voltage,
            max_voltage=args.max_voltage,
            min_energy=args.min_energy,
            max_weight=args.max_weight,
            min_current=args.min_current,
        )
    else:
        print("Analyzed battery: ", args.a_batt)
        battery_analyzer(args.a_batt)


if __name__ == '__main__':
    run()
