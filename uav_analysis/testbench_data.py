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
import json
import numpy
import os
import re
import zipfile

from .components import BATTERIES, PROPELLERS, MOTORS


def parse_fortran_value(value: str) -> Any:
    value = value.strip()
    if value.lower() == '.true.':
        return True
    elif value.lower() == '.false.':
        return False
    elif value.startswith('\''):
        return value.strip('\'')
    elif ',' in value:
        value = value.split(',')
        return [parse_fortran_value(v) for v in value]
    elif 'd' in value or '.' in value:
        return float(value.replace('d', 'e'))
    else:
        return int(value)


RE_BATTERY = re.compile(
    '! *Battery *\((\d+)\) is component named: ([a-zA-Z0-9_]*)')
RE_PROPELLER = re.compile(
    '! *Propeller *\((\d+) uses components named ([a-zA-Z0-9_]*), ([a-zA-Z0-9_]*), ([a-zA-Z0-9_]*)')
RE_WING = re.compile('! *Wing *\((\d+)\) is component named: ([a-zA-Z0-9_]*)')


def parse_fdm_input(lines: Union[str, List[str]]) -> Dict[str, Any]:
    """
    Reads a new_fdm input file and returns the parameters as a dictionary.
    """
    if isinstance(lines, str):
        with open(lines, 'r') as f:
            lines = f.readlines()

    design = dict()
    state = 0
    for line in lines:
        line = line.strip()
        if line == "&aircraft_data":
            assert state == 0
            state = 1
            continue
        elif line == "/":
            assert state == 1
            state = 2
            continue
        elif state != 1:
            continue

        match = RE_BATTERY.match(line)
        if match:
            design['battery(' + match[1] + ').battery_component_key'] \
                = match[2]
            continue

        match = RE_PROPELLER.match(line)
        if match:
            design['propeller(' + match[1] + ').propeller_component_key'] \
                = match[2]
            design['propeller(' + match[1] +
                   ').motor_component_key'] = match[3]
            design['propeller(' + match[1] + ').esc_component_key'] = match[4]
            continue

        match = RE_WING.match(line)
        if match:
            design['wing(' + match[1] + ').wing_component_key'] = match[2]
            continue

        pos = line.find('!')
        if pos >= 0:
            line = line[:pos].strip()
        if not line:
            continue

        pos = line.find('=')
        assert pos >= 0
        attr = line[:pos].strip().replace('%', '.')
        value = line[pos + 1:].strip()

        design[attr] = parse_fortran_value(value)

    if state != 2:
        print("ERROR: Could not load aircraft design")

    return design


class TestbenchData():
    def __init__(self):
        self.byguid = dict()   # Dist[str, Dict[str, Any]]

    def load(self, filename: str):
        byguid = dict()
        components = dict()

        def record(guid: str, data: Dict[str, Any]):
            if guid not in byguid:
                byguid[guid] = dict()
            byguid[guid].update(data)

        with zipfile.ZipFile(filename) as file:
            for name in file.namelist():
                if os.path.basename(name) == 'output.csv':
                    with file.open(name) as content:
                        lines = content.readlines()
                        lines = [line.decode('ascii') for line in lines]
                        for line in csv.DictReader(lines):
                            line = dict(line)
                            record(line['GUID'], line)
                elif os.path.basename(name) in ['FlightDyn.inp',
                                                'FlightDyn_Path1.inp',
                                                'FlightDynamicsPath1.inp']:
                    guid = os.path.basename(os.path.dirname(name))
                    with file.open(name) as content:
                        lines = content.readlines()
                        lines = [line.decode('ascii') for line in lines]
                        data = parse_fdm_input(lines)
                        record(guid, data)
                elif os.path.basename(name) == 'componentMap.json':
                    assert not components
                    with file.open(name) as content:
                        entries = json.loads(content.read())
                        for entry in entries:
                            key = entry['FROM_COMP']
                            val = entry['LIB_COMPONENT']
                            components['components.' + key] = val

                            if key.startswith('Battery_'):
                                components[key + "_Name"] = val
                                components[key + "_Weight"] \
                                    = float(BATTERIES[val]['WEIGHT'])
                                components[key + "_Length"] \
                                    = float(BATTERIES[val]['LENGTH'])
                                components[key + "_Width"] \
                                    = float(BATTERIES[val]['WIDTH'])
                                components[key + "_Thickness"] \
                                    = float(BATTERIES[val]['THICKNESS'])
                            elif key.startswith('Prop_'):
                                components[key + "_Name"] = val
                                components[key + "_Weight"] \
                                    = float(PROPELLERS[val]['Weight'])
                                components[key + "_Diameter"] \
                                    = float(PROPELLERS[val]['DIAMETER'])
                                components[key + "_Thickness"] \
                                    = float(PROPELLERS[val]['HUB_THICKNESS'])
                            elif key.startswith('Motor_'):
                                components[key + "_Name"] = val
                                components[key + "_Weight"] \
                                    = float(MOTORS[val]['WEIGHT'])
                                components[key + "_Length"] \
                                    = float(MOTORS[val]['LENGTH'])
                                components[key + "_TotalLength"] \
                                    = float(MOTORS[val]['TOTAL_LENGTH'])
                                components[key + "_CanLength"] \
                                    = float(MOTORS[val]['CAN_LENGTH'])

        # patch and lookup static values
        for entry in byguid.values():
            entry.update(components)

            extra = dict()
            for key2, val2 in entry.items():
                if not key2.endswith('component_key'):
                    continue
                key3 = key2[:-3] + 'name'
                val3 = components['components.' + val2]
                extra[key3] = val3

                if key2.endswith('.propeller_component_key'):
                    pos = key2.rfind('.')
                    extra[val2 + "_x"] = entry[key2[:pos] + '.x']
                    extra[val2 + "_y"] = entry[key2[:pos] + '.y']
                    extra[val2 + "_z"] = entry[key2[:pos] + '.z']
                elif key2.endswith('.wing_component_key'):
                    pos = key2.rfind('.')
                    extra[val2 + "_x"] = entry[key2[:pos] + '.x']
                    extra[val2 + "_y"] = entry[key2[:pos] + '.y']
                    extra[val2 + "_z"] = entry[key2[:pos] + '.z']

            entry.update(extra)

        self.byguid.update(byguid)

    def save_csv(self, filename: str):
        assert filename.endswith('.csv')
        with open(filename, mode='w') as file:
            writer = csv.DictWriter(file, self.get_fields())
            writer.writeheader()
            for entry in self.byguid.values():
                writer.writerow(entry)

    def has_field(self, field: str) -> bool:
        for entry in self.byguid.values():
            if entry['AnalysisError'] != 'False':
                continue
            if int(entry['Interferences']) != 0:
                continue

            if field not in entry:
                return False

        return True

    def get_fields(self) -> List[str]:
        fields = set()
        for entry in self.byguid.values():
            fields = fields.union(entry.keys())

        return sorted(list(fields))

    def get_tables(self, fields: List[str]) -> Dict[str, numpy.ndarray]:
        result = {field: [] for field in fields}

        for entry in self.byguid.values():
            if entry['AnalysisError'] not in ['False', 'FALSE']:
                continue
            if int(entry['Interferences']) != 0:
                continue

            for field in fields:
                if field in entry:
                    try:
                        entry_value = float(entry[field])
                    except ValueError:
                        entry_value = entry[field]

                    result[field].append(entry_value)
                else:
                    raise ValueError("Unknown field " + field)

        return {key: numpy.array(val) for key, val in result.items()}

    def get_table(self, field: str) -> numpy.ndarray:
        result = self.get_tables([field])
        return result[field]

    def plot2d(self, field1: str, field2: str):
        from matplotlib import pyplot

        tables = self.get_tables([field1, field2])
        fig, ax1 = pyplot.subplots()
        ax1.scatter(
            tables[field1],
            tables[field2],
            s=5.0)
        ax1.set_xlabel(field1)
        ax1.set_ylabel(field2)
        pyplot.show()


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('file', type=str,  nargs='+', metavar='FILE',
                        help="a zip log files to read")
    parser.add_argument('--fields', type=str, metavar='STR', nargs='?', default='',
                        help="print fields containing this filter string")
    parser.add_argument('--components', action='store_true',
                        help="print the list of components for each log file")
    parser.add_argument('--plot', type=str, nargs=2, metavar='VAR',
                        help="plots the given pair of values")
    parser.add_argument('--print', action='store_true',
                        help='print out the first run log data')
    parser.add_argument('--save', type=str, metavar='FILE',
                        help="save the loaded data into a csv file")
    args = parser.parse_args(args)

    data = TestbenchData()
    for file in args.file:
        print("Reading", file)
        data.load(file)

    if args.fields != '':
        fields = data.get_fields()
        if args.fields != None:
            fields = [field for field in fields if args.fields in field]
        print("fields:")
        for field in fields:
            print(field)

    if args.components:
        print(json.dumps(data.componentmap_json, indent=2, sort_keys=True))

    if args.plot:
        data.plot2d(args.plot[0], args.plot[1])

    if args.print:
        guid = list(data.byguid.keys())[0]
        print(json.dumps(data.byguid[guid], indent=2, sort_keys=True))

    if args.save:
        print("Writing to", args.save)
        data.save_csv(args.save)


if __name__ == '__main__':
    run()
