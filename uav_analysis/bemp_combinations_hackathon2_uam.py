#!/usr/bin/env python3
# Copyright (C) 2021, Miklos Maroti
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

from typing import List, Dict, Any, Union

import csv
import os
import sys


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


def is_reversible_propeller(name: str) -> bool:
    assert name in PROPELLERS
    return name + 'P' in PROPELLERS


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


def battery_motor_propeller_generator(reversible: bool = True):
    for battery in BATTERIES:
        for motor in MOTORS:
            for propeller in PROPELLERS:
                if reversible and not is_reversible_propeller(propeller):
                    continue
                if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                    continue

                yield get_bemp_data(battery, motor, propeller)


def motor_propeller_generator(reversible: bool = True):
    for motor in MOTORS:
        for propeller in PROPELLERS:
            if reversible and not is_reversible_propeller(propeller):
                continue
            if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                continue

            yield get_bemp_data(None, motor, propeller)


def save_to_csv(generator, filename: str):
    with open(filename, 'w', newline='') as file:
        row = generator.__next__()
        writer = csv.DictWriter(file, row.keys())
        writer.writeheader()
        writer.writerow(row)
        for row in generator:
            writer.writerow(row)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--save', type=str, metavar='FILE',
                        help="save the loaded data into a csv file")
    args = parser.parse_args(args)

    generator = battery_motor_propeller_generator(reversible=False)
    if args.save:
        print("Writing to", args.save)
        save_to_csv(generator, args.save)
    else:
        for entry in generator:
            print(entry)
