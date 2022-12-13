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
import os
from multiprocessing.pool import ThreadPool
from typing import Dict, List

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem, StarmapParallelization
from pymoo.core.variable import Real
from pymoo.optimize import minimize

from .tweak_fdm_input import CONTROL, FDM_PATH, parse_fdm_output, run_new_fdm

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
            value = float(match.group(4))
            param = match.group(2)

            params[param.strip()] = value

    return params


class FDMControlTweak(ElementwiseProblem):
    def __init__(
        self,
        input_file,
        control_multipliers: List[float],
        target: str,
        fdm_binary: str,
        **kwargs,
    ):

        with open(input_file) as f:
            fdm_input_lines = f.readlines()

        params = get_control_coeffs(fdm_input_lines)
        self.fdm_input_lines = fdm_input_lines
        self.target = target
        self.fdm_input_lines = fdm_input_lines
        self.fdm_binary = fdm_binary
        self.input_path = os.path.dirname(os.path.abspath(input_file))
        param_vars = {}
        xls = []
        xus = []

        for param, value in params.items():
            param_vars[param] = Real(
                bounds=[
                    xl := value / control_multipliers["QR".index(param[0])],
                    xu := value * control_multipliers["QR".index(param[0])],
                ]
            )

            xls.append(xl)
            xus.append(xu)

        super().__init__(
            n_constr=0, n_var=5, n_obj=1, xl=xls, xu=xus, vars=param_vars, **kwargs
        )
        self.params = list(param_vars.keys())

    def _evaluate(self, x, out, *args, **kwargs):
        param_values = {param: x[i] for i, param in enumerate(self.params)}

        new_input = ""

        for line in self.fdm_input_lines:
            if match := CONTROL.match(line):
                param = match.group(2)
                new_value = param_values[param.strip()]
                new_input += match.group(1) + str(new_value) + "\n"
            else:
                new_input += line

        fdm_out = run_new_fdm(
            fdm_binary=self.fdm_binary, fdm_input=new_input, path=self.input_path
        )

        fdm_scores = parse_fdm_output(fdm_out)
        print(fdm_scores)
        out["F"] = (
            -1 * fdm_scores["path_score" if self.target == "score" else self.target]
        )


def optimize_fdm(
    fdm_input: str,
    control_coeff: List[float],
    target: str,
    nproc: int,
    fdm_binary: str,
    pop_size: int = 50,
    n_gen: int = 50,
):
    pool = ThreadPool(processes=nproc)
    runner = StarmapParallelization(pool.starmap)

    problem = FDMControlTweak(
        fdm_input,
        control_coeff,
        target,
        fdm_binary=fdm_binary,
        elementwise_runner=runner,
    )

    algorithm = NSGA2(pop_size=pop_size, termination=("n_gen", n_gen), verbose=True)

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

    parser.add_argument(
        "--fdm",
        default=os.path.join(FDM_PATH, "new_fdm_nolog"),
        metavar="PATH",
        help="path to fdm executable",
    )

    parser.add_argument(
        "--nproc", default=8, metavar="NPROC", help="the number of processes"
    )

    parser.add_argument(
        "--pop-size", default=50, metavar="POP_SIZE", help="the population size"
    )

    parser.add_argument(
        "--n-gen", default=50, metavar="N_GEN", help="the number generations"
    )

    args = parser.parse_args(args)

    optimize_fdm(
        args.input,
        args.control_coef,
        target=args.optimize,
        nproc=args.nproc,
        fdm_binary=args.fdm,
        pop_size=args.pop_size,
        n_gen=args.n_gen,
    )


if __name__ == "__main__":
    run()
