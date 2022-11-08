#!/usr/bin/env python3
# Copyright (C) 2021, Miklos Maroti and Carlos Olea
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

from typing import Dict, Any, Tuple

import csv
import json
import math
import os

DATAPATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                        'data_hackathon2')


def load_static_data(filename: str, modelkey: str = 'MODEL_NAME') -> Dict[str, Dict[str, Any]]:
    filename = os.path.join(DATAPATH, filename)

    result = dict()
    with open(filename) as file:
        if filename.endswith('.csv'):
            reader = csv.DictReader(file)
            for line in reader:
                result[line[modelkey]] = dict(line)
        elif filename.endswith('.json'):
            result = json.load(file)
        else:
            raise ValueError("Invalid filename extension")

    return result


BATTERIES = load_static_data('Battery.csv')
PROPELLERS = load_static_data('PropellerExt.csv')
MOTORS = load_static_data('Motor.csv')
WINGS = load_static_data('aero_info.json')


def find_rpm_minmax(perf_file: str) -> Tuple[float, float]:
    filename = os.path.join(DATAPATH, "..", "prop_tables", perf_file)
    rpm_min = 1e10
    rpm_max = -1e10
    with open(filename, 'r') as file:
        for line in file.readlines():
            line = line.strip()
            if line.startswith("PROP RPM = "):
                rpm = rpm = float(line.split()[3])
                if math.isfinite(rpm):
                    rpm_min = min(rpm_min, rpm)
                    rpm_max = max(rpm_max, rpm)
    assert rpm_min != 1e10
    return (rpm_min, rpm_max)


def create_extended_propeller_table():
    filename = os.path.join(DATAPATH, "PropellerExt.csv")
    with open(filename, "w") as file:
        writer = None

        for data in PROPELLERS.values():
            rpm_min, rpm_max = find_rpm_minmax(data['Performance_File'])
            data["RPM_MIN"] = rpm_min
            data["RPM_MAX"] = rpm_max

            if writer is None:
                writer = csv.DictWriter(file, fieldnames=data.keys())
                writer.writeheader()
            writer.writerow(data)


if __name__ == '__main__':
    create_extended_propeller_table()
