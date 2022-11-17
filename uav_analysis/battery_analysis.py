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

from typing import Any, Dict, Iterable, Generator, Optional

import csv
import json
import sys

from .components import BATTERIES


def battery_generator(max_series: int,
                      max_parallel: int,
                      max_voltage: Optional[float] = None,
                      battery_name: Optional[str] = None,
                      ) -> Generator[Dict[str, Any], None, None]:

    for battery in BATTERIES.values():
        name = battery["MODEL"]
        if battery_name and name != battery_name:
            continue

        voltage = float(battery["VOLTAGE"])                     # V
        capacity = float(battery["CAPACITY"]) * 1e-3            # Ah from mAh
        weight = float(battery["WEIGHT"])                       # kg
        length = float(battery["LENGTH"]) * 1e-3                # m from mm
        width = float(battery["WIDTH"]) * 1e-3                  # m from mm
        thickness = float(battery["THICKNESS"]) * 1e-3          # m from mm
        volume = length * width * thickness                     # m^3
        discharge_rate = float(battery["CONT_DISCHARGE_RATE"])  # C

        for series in range(1, max_series + 1):
            if max_voltage and voltage * series > max_voltage:
                break

            for parallel in range(1, max_parallel + 1):
                yield {
                    "name": name,
                    "series_count": series,
                    "parallel_count": parallel,
                    "discharge_rate": discharge_rate,                 # C
                    "single_length": length,                          # m
                    "single_width": width,                            # m
                    "single_thickness": thickness,                    # m
                    "single_voltage": voltage,                        # V
                    "single_capacity": capacity,                      # Ah
                    "single_weight": weight,                          # kg
                    "single_volume": volume,                          # m^3
                    "single_current": capacity * discharge_rate,      # A
                    "total_voltage": voltage * series,
                    "total_capacity": capacity * parallel,
                    "total_weight": weight * series * parallel,
                    "total_volume": volume * series * parallel,
                    "total_current": capacity * parallel * discharge_rate,
                }


def save_to_csv(iterable: Iterable[Dict[str, Any]], output: str):
    with open(output, 'w', newline='') as file:
        count = 0
        writer = None
        for row in iterable:
            if writer is None:
                writer = csv.DictWriter(file, row.keys())
                writer.writeheader()
            writer.writerow(row)
            if count % 100 == 0:
                sys.stdout.write("\rGenerated: {}".format(count))
                sys.stdout.flush()
            count += 1

    sys.stdout.write("\rGenerated: {}".format(count))
    print("\nResults saved to", output)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--battery', metavar="NAME",
                        help="limits the search to this battery")
    parser.add_argument('--max-voltage', type=float, metavar="VOLT", default=None,
                        help="maximum voltage of the pack")
    parser.add_argument('--max-series', type=int, metavar="NUM", default=1,
                        help="maximum series count")
    parser.add_argument('--max-parallel', type=int, metavar="NUM", default=1,
                        help="maximum parallel count")
    parser.add_argument('--output', metavar="FILE",
                        help="save data to this CSV file")

    args = parser.parse_args(args)

    generator = battery_generator(
        max_series=args.max_series,
        max_parallel=args.max_parallel,
        max_voltage=args.max_voltage,
        battery_name=args.battery,
    )

    if args.output:
        save_to_csv(generator, args.output)
    else:
        for row in generator:
            print(json.dumps(row, indent=2))


if __name__ == '__main__':
    run()
