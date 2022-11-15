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

from . import testbench_data
from . import motor_propeller_analysis
from . import battery_analysis
from . import wing_analysis
from . import aggregate_analysis
from . import napkin_calculator
from . import symbolic_propeller
from . import tweak_fdm_input


def run():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', metavar="COMMAND", help=", ".join(sorted([
        "testbench-data",
        "motor-propeller-analysis",
        "motor-propeller-single",
        "motor-propeller-approximate",
        "battery-analysis",
        "wing-analysis",
        "aggregate-analysis",
        "napkin-calculator",
        "symbolic-propeller",
        'tweak-fdm-input',
    ])))
    args = parser.parse_args(sys.argv[1:2])

    # hack the program name for nested parsers
    sys.argv[0] += ' ' + args.command
    args.command = args.command.replace('_', '-')

    if args.command == 'testbench-data':
        testbench_data.run(args=sys.argv[2:])
    elif args.command == 'motor-propeller-analysis':
        motor_propeller_analysis.run(args=sys.argv[2:])
    elif args.command == 'motor-propeller-single':
        motor_propeller_analysis.run_single(args=sys.argv[2:])
    elif args.command == 'motor-propeller-approximate':
        motor_propeller_analysis.run_approximate(args=sys.argv[2:])
    elif args.command == 'battery-analysis':
        battery_analysis.run(args=sys.argv[2:])
    elif args.command == 'wing-analysis':
        wing_analysis.run(args=sys.argv[2:])
    elif args.command == 'aggregate-analysis':
        aggregate_analysis.run(args=sys.argv[2:])
    elif args.command == 'napkin-calculator':
        napkin_calculator.run(args=sys.argv[2:])
    elif args.command == 'symbolic-propeller':
        symbolic_propeller.run(args=sys.argv[2:])
    elif args.command == 'tweak-fdm-input':
        tweak_fdm_input.run(args=sys.argv[2:])
    else:
        parser.print_help()


if __name__ == '__main__':
    run()
