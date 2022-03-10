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

import argparse
import sys

from click import command

from uav_analysis import bemp_combinations_hackathon1, bemp_fdm_generator_hackathon1, mass_properties_hackathon1, testbench_data
from uav_analysis import bemp_combinations_hackathon2_uam, bemp_fdm_generator_hackathon2_uam, battery_analysis


def run():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', help="""
    testbench-data, mass-properties-hackathon1, bemp-combinations-hackathon1, bemp-fdm-generator-hackathon1, 
    bemp-combinations-hackathon2, bemp-fdm-generator-hackathon2
    """)
    args = parser.parse_args(sys.argv[1:2])

    # hack the program name for nested parsers
    sys.argv[0] += ' ' + args.command
    args.command = args.command.replace('_', '-')

    if args.command == 'testbench-data':
        testbench_data.run(args=sys.argv[2:])
    elif args.command == 'mass-properties-hackathon1':
        mass_properties_hackathon1.run(args=sys.argv[2:])
    elif args.command == 'bemp-combinations-hackathon1':
        bemp_combinations_hackathon1.run(args=sys.argv[2:])
    elif args.command == 'bemp-fdm-generator-hackathon1':
        bemp_fdm_generator_hackathon1.run(args=sys.argv[2:])
    elif args.command == 'bemp-combinations-hackathon2':
        bemp_combinations_hackathon2_uam.run(args=sys.argv[2:])
    elif args.command == 'bemp-fdm-generator-hackathon2':
        bemp_fdm_generator_hackathon2_uam.run(args=sys.argv[2:])
    elif args.command == 'battery-analysis':
        battery_analysis.run(args=sys.argv[2:])
    else:
        parser.print_help()


if __name__ == '__main__':
    run()
