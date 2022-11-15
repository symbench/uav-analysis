#!/usr/bin/env python3
# Copyright (C) 2021, Carlos Olea and Miklos Maroti
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

from typing import Any, Dict, Iterable, List, Optional, Tuple, Generator

import concurrent.futures
import csv
import math
import os
import subprocess
import sys
import re
import random

from .components import DATAPATH

PROPELLER = re.compile(
    r"^(\s*propeller\(\d*\)\%([xyz])\s*=\s*)([+-]?\d+\.?|[+-]?\d*\.\d+|[+-]?\d*\.\d+e[+-]\d+)\s*$")

WING = re.compile(
    r"^(\s*wing\(\d*\)\%([xyz])\s*=\s*)([+-]?\d+\.?|[+-]?\d*\.\d+|[+-]?\d*\.\d+e[+-]\d+)\s*$")


def find_tweakes(fdm_input: List[str]) -> Dict[str, float]:
    values = dict()
    for line in fdm_input:
        for pat in [PROPELLER, WING]:
            match = pat.match(line)
            if match:
                values[match.group(1)] = float(match.group(2))
    return values


def rewrite_line(line: str,
                 prop_delta: Tuple[float, float, float],
                 wing_delta: Tuple[float, float, float]) -> str:
    match = PROPELLER.match(line)
    if match:
        old_value = float(match.group(3))
        new_delta = prop_delta["xyz".index(match.group(2))]
        new_value = old_value + random.randint(-10, 10) * 0.1 * new_delta
        return match.group(1) + str(new_value) + "\n"

    match = WING.match(line)
    if match:
        old_value = float(match.group(3))
        new_delta = wing_delta["xyz".index(match.group(2))]
        new_value = old_value + random.randint(-10, 10) * 0.1 * new_delta
        return match.group(1) + str(new_value) + "\n"

    return line


def create_fdm_input(lines: List[str],
                     prop_delta: Tuple[float, float, float],
                     wing_delta: Tuple[float, float, float],
                     ) -> str:
    fdm_input = ""
    for line in lines:
        fdm_input += rewrite_line(line, prop_delta, wing_delta)
    return fdm_input


def run_new_fdm(fdm_binary: str, path: str, fdm_input: str) -> str:
    proc = subprocess.Popen(
        fdm_binary,
        cwd=path,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE)
    fdm_output, _ = proc.communicate(input=fdm_input.encode())
    if proc.returncode != 0:
        raise KeyboardInterrupt
    return fdm_output.decode()


def parse_fdm_output(fdm_output: str) -> Dict[str, float]:
    num_trims = 0
    max_speed = -1.0
    min_pitch = 90.0
    avg_drag = 0.0
    avg_current = 0.0
    avg_power = 0.0

    lines = fdm_output.splitlines()
    active = False
    for line in lines:
        if line.startswith("  Lateral speed  Distance  Flight time  Pitch angle  Max uc    Thrust      Lift       Drag    Current  Total power Frac amp  Frac pow  Frac current"):
            active = True
        elif line == "":
            active = False
        elif active:
            if line.startswith("      (m/s)") or line[11:23] == "            ":
                continue

            num_trims += 1
            # fortran uses fixed widths
            max_speed = max(max_speed, float(line[0:11]))
            min_pitch = min(min_pitch, float(line[35:47]))
            avg_drag += float(line[80:91])
            avg_current += float(line[91:102])
            avg_power += float(line[102:113])

    return {
        "num_trims": num_trims,
        "max_speed": max_speed,
        "min_pitch": min_pitch,
        "avg_drag": avg_drag / max(num_trims, 1),
        "avg_current": avg_current / max(num_trims, 1),
        "avg_power": avg_power / max(num_trims, 1),
    }


def is_better(old_result, new_result) -> bool:
    if old_result is None:
        return True

    if new_result["num_trims"] != old_result["num_trims"]:
        return new_result["num_trims"] > old_result["num_trims"]

    if new_result["max_speed"] != old_result["max_speed"]:
        return new_result["max_speed"] > old_result["max_speed"]

    return new_result["avg_power"] < old_result["avg_power"]


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--fdm',
                        default=os.path.relpath(os.path.join(
                            DATAPATH, '..', '..', 'flight-dynamics-model', 'bin', 'new_fdm_nolog')),
                        metavar='PATH', help="path to fdm executable")
    parser.add_argument('--input', metavar='FILE', type=str,
                        default="flightDynFast.inp",
                        help='sets the input filenam')
    parser.add_argument('--runs', type=int, default=100,
                        help='sets the number of runs')
    parser.add_argument('--prop-delta', metavar='N', type=float, nargs=3,
                        default=[0, 0, 0],
                        help='sets the x, y and z maximum deltas for the propellers')
    parser.add_argument('--wing-delta', metavar='N', type=float, nargs=3,
                        default=[0, 0, 0],
                        help='sets the x, y and z maximum deltas for the wings')

    args = parser.parse_args(args)

    with open(args.input, 'r') as f:
        lines = f.readlines()
        fdm_binary = os.path.abspath(args.fdm)
        input_path = os.path.dirname(os.path.abspath(args.input))

        def task(index):
            fdm_input = create_fdm_input(
                lines, args.prop_delta, args.wing_delta)
            fdm_output = run_new_fdm(fdm_binary, input_path, fdm_input)
            result = parse_fdm_output(fdm_output)
            result["fdm_input"] = fdm_input
            return result

        best = None
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            results = executor.map(task, range(args.runs))
            for result in results:
                if is_better(best, result):
                    best = dict(result)
                    with open(args.input + ".best", 'w') as f:
                        f.write(best["fdm_input"])
                    result["best"] = True
                else:
                    result["best"] = False

                del result["fdm_input"]
                print(result)


if __name__ == '__main__':
    run()
