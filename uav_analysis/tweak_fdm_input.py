#!/usr/bin/env python3
# Copyright (C) 2022, Miklos Maroti
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

from typing import Dict, List, Tuple

import concurrent.futures
import os
import subprocess
import re
import random

from .components import FDM_PATH

PROPELLER = re.compile(
    r"^(\s*propeller\(\d*\)\%([xyz])\s*=\s*)([-+0-9.e]*)\s*$")

WING = re.compile(
    r"^(\s*wing\(\d*\)\%([xyz])\s*=\s*)([-+0-9.e]*)\s*$")

CONTROL = re.compile(
    r"^(\s*control\%([QR])[_a-zA-Z]*\s*=\s*)([-+0-9.e]*)\s*$")


def rewrite_line(line: str,
                 prop_delta: Tuple[float, float, float],
                 wing_delta: Tuple[float, float, float],
                 control_coef: Tuple[float, float],
                 ) -> str:

    if match := PROPELLER.match(line):
        old_value = float(match.group(3))
        new_delta = prop_delta["xyz".index(match.group(2))]
        new_value = old_value + random.randint(-10, 10) * 0.1 * new_delta
        return match.group(1) + str(new_value) + "\n"

    if match := WING.match(line):
        old_value = float(match.group(3))
        new_delta = wing_delta["xyz".index(match.group(2))]
        new_value = old_value + random.randint(-10, 10) * 0.1 * new_delta
        return match.group(1) + str(new_value) + "\n"

    if match := CONTROL.match(line):
        old_value = float(match.group(3))
        new_coef = control_coef["QR".index(match.group(2))]
        new_value = old_value * new_coef ** (random.randint(-10, 10) * 0.1)
        return match.group(1) + str(new_value) + "\n"

    return line


def create_fdm_input(lines: List[str],
                     prop_delta: Tuple[float, float, float],
                     wing_delta: Tuple[float, float, float],
                     control_coef: Tuple[float, float],
                     ) -> str:
    fdm_input = ""
    for line in lines:
        fdm_input += rewrite_line(line, prop_delta, wing_delta, control_coef)
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


TOTAL_SCORE = re.compile(
    r"^\s*Path_traverse_score_based_on_requirements\s*([-+0-9.]*)\s*$")

TOTAL_DIST = re.compile(
    r"^\s*Flight_distance\s*([-+0-9.]*)\s*$")


def parse_fdm_output(fdm_output: str) -> Dict[str, float]:
    num_trims = 0
    max_speed = -1.0
    min_pitch = 90.0
    avg_drag = 0.0
    avg_current = 0.0
    avg_power = 0.0
    total_score = 0.0
    total_dist = 0.0

    active = False
    for line in fdm_output.splitlines():
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
        elif match := TOTAL_SCORE.match(line):
            total_score = float(match.group(1))
        elif match := TOTAL_DIST.match(line):
            total_dist = float(match.group(1))

    return {
        "num_trims": num_trims,
        "max_speed": max_speed,
        "min_pitch": min_pitch,
        "avg_drag": round(avg_drag / max(num_trims, 1), 2),
        # "avg_current": round(avg_current / max(num_trims, 1), 2),
        "avg_power": round(avg_power / max(num_trims, 1), 2),
        'path_dist': round(total_dist, 2),
        'path_score': total_score,
    }


def is_better_trims(old_result, new_result) -> bool:
    if old_result is None:
        return True

    if new_result["num_trims"] != old_result["num_trims"]:
        return new_result["num_trims"] > old_result["num_trims"]

    if new_result["max_speed"] != old_result["max_speed"]:
        return new_result["max_speed"] > old_result["max_speed"]

    return new_result["avg_power"] < old_result["avg_power"]


def is_better_score(old_result, new_result) -> bool:
    if old_result is None:
        return True

    if new_result["path_score"] != old_result["path_score"]:
        return new_result["path_score"] > old_result["path_score"]

    if new_result["num_trims"] != old_result["num_trims"]:
        return new_result["num_trims"] > old_result["num_trims"]

    return new_result["avg_power"] < old_result["avg_power"]


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--fdm',
                        default=os.path.join(FDM_PATH, 'new_fdm_nolog'),
                        metavar='PATH', help="path to fdm executable")
    parser.add_argument('--input', metavar='FILE', type=str,
                        default="flightDynFast.inp",
                        help='sets the input filenam')
    parser.add_argument('--output', metavar="FILE", type=str,
                        default='tweaked.inp',
                        help="sets the optimized input filename")
    parser.add_argument('--nproc', type=int, default=8,
                        help='number of processes')
    parser.add_argument('--runs', type=int, default=100,
                        help='sets the number of runs')
    parser.add_argument('--prop-delta', metavar='N', type=float, nargs=3,
                        default=[0, 0, 0],
                        help='sets the propeller x, y and z maximum deltas')
    parser.add_argument('--wing-delta', metavar='N', type=float, nargs=3,
                        default=[0, 0, 0],
                        help='sets the wing x, y and z maximum deltas')
    parser.add_argument('--control-coef', metavar='N', type=float, nargs=2,
                        default=[1, 1],
                        help='sets the controller Q and R maximum multipliers')
    parser.add_argument("--optimize", type=str, default="trims",
                        choices=["trims", "score"],
                        help="sets the optimization type")

    args = parser.parse_args(args)

    fdm_binary = os.path.abspath(args.fdm)
    input_path = os.path.dirname(os.path.abspath(args.input))
    is_better = is_better_trims if args.optimize == "trims" \
        else is_better_score

    with open(args.input, 'r') as f:
        lines = f.readlines()

        def task(index):
            fdm_input = create_fdm_input(
                lines,
                args.prop_delta,
                args.wing_delta,
                args.control_coef,
            )
            fdm_output = run_new_fdm(fdm_binary, input_path, fdm_input)
            result = parse_fdm_output(fdm_output)
            result["fdm_input"] = fdm_input
            result["fdm_output"] = fdm_output
            return result

        def print_result(result):
            result = dict(result)
            del result["fdm_input"]
            del result["fdm_output"]
            print(result)

        best = None
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.nproc) as executor:
            results = executor.map(task, range(args.runs))
            for result in results:
                if is_better(best, result):
                    with open(args.output, 'w') as f:
                        f.write(result["fdm_input"])
                    with open(os.path.splitext(args.output)[0] + ".out", 'w') as f:
                        f.write(result["fdm_output"])
                    result["best"] = True
                    best = result
                else:
                    result["best"] = False

                print_result(result)

        print("\nBest was:")
        print_result(best)


if __name__ == '__main__':
    run()
