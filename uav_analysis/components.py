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

FDM_PATH = default = os.path.relpath(os.path.join(
    DATAPATH, '..', '..', 'flight-dynamics-model', 'bin'))

PROP_PATH = os.path.abspath(os.path.join(DATAPATH, "..", "PropData"))


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
AERO_INFO = load_static_data('aero_info.json')


def find_rpm_j_minmax(perf_file: str) -> Tuple[float, float, float, float]:
    filename = os.path.join(PROP_PATH, perf_file)
    rpms = []
    js = []
    with open(filename, 'r') as file:
        for line in file.readlines():
            line = line.strip()
            if line.startswith("PROP RPM = "):
                rpm = float(line.split()[3])
                if math.isfinite(rpm):
                    rpms.append(rpm)
            elif len(rpms) == 1 and line and line[:1].isdigit():
                j = float(line.split()[1])
                if math.isfinite(j):
                    js.append(j)
    return (min(rpms), max(rpms), min(js), max(js))


def create_extended_propeller_table():
    filename = os.path.join(DATAPATH, "PropellerExt.csv")
    with open(filename, "w") as file:
        writer = None

        for data in PROPELLERS.values():
            rpm_min, rpm_max, j_min, j_max = find_rpm_j_minmax(
                data['Performance_File'])
            data["RPM_MIN"] = rpm_min
            data["RPM_MAX"] = rpm_max
            data["J_MIN"] = j_min
            data["J_MAX"] = j_max

            if writer is None:
                writer = csv.DictWriter(file, fieldnames=data.keys())
                writer.writeheader()
            writer.writerow(data)


if __name__ == '__main__':
    # print(find_rpm_j_minmax("PER3_12x38SF.dat"))
    create_extended_propeller_table()
