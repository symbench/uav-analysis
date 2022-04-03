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
import math
import sys

from .components import BATTERIES

"""
The battery pack is needs to deliver a relatively fixed voltage, large
enough to optimally drive the motor, but not too much so that it is not
over the power/current limit of the motor. So we need to specify a max
voltage and also try to maximize the voltage below that, which involves
setting the series count. The battery pack maximizes its total energy,
the current (The Vitaly Beta cannot give enough instantaneous current),
while minimizing its weight and volume. We are not limited in practice 
by volume, so we maximize the energy and current per weight.

Based on the pareto front and 940 max voltage, there are only 3 options:
- Vitaly Beta (not enough current),
- Tattu 25Ah Li (the overall winner),
- Eagle 11 Ah Li (slightly better voltage, but not enough current).
"""


def series_generator(max_voltage: float,
                     battery: Optional[str] = None,
                     ) \
        -> Generator[Dict[str, Any], None, None]:

    for batt in BATTERIES.values():
        name = batt["Name"]
        if battery and name != battery:
            continue

        voltage = float(batt["Min Voltage [V]"])
        capacity = float(batt["Capacity [Ah]"])
        discharge_rate = float(batt["Cont. Discharge Rate [C]"])
        weight = float(batt["Weight [kg]"])
        volume = float(batt["Volume [mm^3]"]) * 1e-9

        for series in range(1, int(math.floor(max_voltage / voltage)) + 1):
            yield {
                "name": name,
                "series_count": series,
                "voltage": voltage * series,
                "capacity": capacity,
                "current": capacity * discharge_rate,
                "weight": weight * series,
                "volume": volume * series,
                "energy": voltage * capacity * series,
                "energy_per_weight": voltage * capacity / weight,
                "current_per_weight": capacity * discharge_rate / (weight * series),
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
                sys.stdout.write("\r{}".format(count))
                sys.stdout.flush()
            count += 1

    print("\nResults saved to", output)


def print_json(iterable: Iterable[Dict[str, Any]]):
    for row in iterable:
        print(json.dumps(row, indent=2))


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--battery', metavar="NAME",
                        help="limits the search to this battery")
    parser.add_argument('--max-voltage', type=float, metavar="VOLT",
                        default=1000,
                        help="maximum voltage of the pack")
    parser.add_argument('--output', metavar="FILE",
                        help="save data to this CSV file")

    args = parser.parse_args(args)

    generator = series_generator(
        max_voltage=args.max_voltage,
        battery=args.battery)

    if args.output:
        save_to_csv(generator, args.output)
    else:
        print_json(generator)


if __name__ == '__main__':
    run()
