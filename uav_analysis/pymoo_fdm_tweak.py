#!/usr/bin/env python3
# Copyright (C) 2022, Umesh Timalsina
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
from typing import Dict, List

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem
from pymoo.core.variable import Real
from pymoo.optimize import minimize

from .tweak_fdm_input import CONTROL

# Variables
# Q_position
# Q_velocity
# Q_angular_velocity
# Q_angles
# R


def get_control_coeffs(lines: List[str]) -> Dict[str, float]:
    params = {}
    for line in lines:
        if match := CONTROL.match(line):
            print(match)
            value = float(match.group(4))
            param = match.group(2)

            params[param.strip()] = value

    return params


class FDMControlTweak(ElementwiseProblem):
    def __init__(self, fdm_input_lines: List[str], control_multipliers: List[int, int]):
        params = get_control_coeffs(fdm_input_lines)
        self.fdm_input_lines = fdm_input_lines
        param_vars = {}

        for param, value in params.items():
            param_vars[param] = Real(
                bounds=[
                    value / control_multipliers.index(param[0]),
                    value * control_multipliers.index(param[0]),
                ]
            )

        super().__init__(n_constr=0, n_var=5, vars=vars)

    def _evaluate(self, x, out, *args, **kwargs):
        print(x)


def optimize_fdm(lines: List[str], control_coeff: List[int, int], target: str):
    problem = FDMControlTweak(lines, target)

    algorithm = NSGA2(pop_size=100, termination=("n_gen", 100), verbose=True)

    result = minimize(problem, algorithm)


def run(args=None):
    from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

    parser = ArgumentParser(
        description="FDM Parameter tweaks with pymoo",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--input",
        metavar="FILE",
        type=str,
        default="flightDynFast.inp",
        help="sets the input filename",
    )

    parser.add_argument(
        "--control-coef",
        metavar="N",
        type=float,
        nargs=2,
        default=[1, 1],
        help="sets the controller Q and R maximum multipliers",
    )

    parser.add_argument(
        "--optimize",
        type=str,
        default="score",
        choices=["score"],
        help="sets the optimization type",
    )

    args = parser.parse_args(args)

    with open(args.input) as f:
        lines = f.readlines()
        optimize_fdm(lines, args.control_coeff, target=args.optimize)


if __name__ == "__main__":
    run()
