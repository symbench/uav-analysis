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

from gettext import find
from typing import Dict, Any, Tuple

import csv
import math
import os

DATAPATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                        'data_demo1')


def load_static_data(name: str) -> Dict[str, Dict[str, Any]]:
    filename = os.path.join(DATAPATH, name + '.csv')

    result = dict()
    with open(filename) as file:
        reader = csv.DictReader(file)
        for line in reader:
            result[line['Name']] = dict(line)

    return result


BATTERIES = load_static_data('Battery')
PROPELLERS = load_static_data('PropellerExt')
WINGS = load_static_data('Wing')
MOTORS = load_static_data('Motor')


def find_rpm_minmax(propname: str) -> Tuple[float, float]:
    filename = os.path.join(DATAPATH, "propeller", propname + ".dat")
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
    import csv

    filename = os.path.join(DATAPATH, "PropellerExt.csv")
    with open(filename, "w") as file:
        writer = None

        for prop, data in PROPELLERS.items():
            rpm_min, rpm_max = find_rpm_minmax(prop)
            data["RPM Min"] = rpm_min
            data["RPM Max"] = rpm_max

            if writer is None:
                writer = csv.DictWriter(file, fieldnames=data.keys())
                writer.writeheader()
            writer.writerow(data)


def get_bemp_data(battery: str, motor: str, propeller: str) -> Dict[str, float]:
    data = dict()
    if battery is not None:
        for key, val in BATTERIES[battery].items():
            data['Battery.' + key] = val
    for key, val in MOTORS[motor].items():
        data['Motor.' + key] = val
    for key, val in PROPELLERS[propeller].items():
        data['Propeller.' + key] = val
    return data


if __name__ == '__main__':
    create_extended_propeller_table()
