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

from typing import Dict, Any

import csv
import os


def load_static_data(name: str) -> Dict[str, Dict[str, Any]]:
    filename = os.path.abspath(os.path.dirname(__file__))
    filename = os.path.join(filename, 'data_hackathon2_uam', name + '.csv')

    result = dict()
    with open(filename) as file:
        reader = csv.DictReader(file)
        for line in reader:
            result[line['Name']] = dict(line)

    return result


BATTERIES = load_static_data('Battery')
PROPELLERS = load_static_data('Propeller')
WINGS = load_static_data('Wing')
MOTORS = load_static_data('Motor')

BATTERY_NAMES = sorted(BATTERIES.keys())
PROPELLER_NAMES = sorted(BATTERIES.keys())
WING_NAMES = sorted(WINGS.keys())
MOTOR_NAMES = sorted(MOTORS.keys())


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


def battery_motor_propeller_generator():
    for battery in BATTERIES:
        for motor in MOTORS:
            for propeller in PROPELLERS:
                if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                    continue

                yield get_bemp_data(battery, motor, propeller)


def motor_propeller_generator():
    for motor in MOTORS:
        for propeller in PROPELLERS:
            if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                continue

            yield get_bemp_data(None, motor, propeller)
