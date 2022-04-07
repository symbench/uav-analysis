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

from typing import Dict, List, Tuple, Optional

import collections
import math
import matplotlib.pyplot as plt
import numpy
import sympy

from . import func_approx

Entry = collections.namedtuple('Entry', ['rpm', 'j', 'ct', 'cp'])


def read_propeller_table(filename: str) -> List[Entry]:
    table = []
    with open(filename, 'r') as file:
        rpm = None
        for line in file.readlines():
            line = line.strip()
            if line.startswith("PROP RPM = "):
                line = line.split()
                rpm = float(line[3])
                if not math.isfinite(rpm):
                    rpm = None
            elif rpm is not None and line and line[:1].isdigit():
                line = line.replace('-NaN', ' -NaN')
                line = line.split()
                j = float(line[1])
                ct = float(line[3])
                cp = float(line[4])
                if math.isfinite(j) and math.isfinite(ct) and math.isfinite(cp):
                    table.append(Entry(rpm, j, ct, cp))
    return table


def interpolate_table(table: List[Entry]) -> Dict[str, numpy.ndarray]:
    rpms = collections.Counter()
    for entry in table:
        rpms[entry.rpm] += 1

    rpms = sorted([rpm for rpm, cnt in rpms.items() if cnt >= 3])
    assert len(rpms) >= 2

    js = sorted([entry.j for entry in table if entry.rpm == rpms[0]])
    assert len(js) == len(set(js))

    cts = numpy.empty((len(rpms), len(js)), dtype=float)
    cps = numpy.empty((len(rpms), len(js)), dtype=float)
    for idx, rpm in enumerate(rpms):
        row = numpy.array([(entry.j, entry.ct, entry.cp)
                           for entry in table if entry.rpm == rpm])
        cts[idx, :] = numpy.interp(js, row[:, 0], row[:, 1])
        cps[idx, :] = numpy.interp(js, row[:, 0], row[:, 2])

    return {
        "rpms": numpy.array(rpms, dtype=float),  # [rpm]
        "js": numpy.array(js, dtype=float),      # [j]
        "cts": cts,                              # [rpm, j]
        "cps": cps,                              # [rpm, j]
    }


def approximate_table(table: List[Entry]) -> sympy.Expr:
    rpms = numpy.array([entry.rpm for entry in table])
    js = numpy.array([entry.j for entry in table])
    cts = numpy.array([entry.ct for entry in table])
    cps = numpy.array([entry.cp for entry in table])

    rpm = sympy.Symbol("rpm")
    j = sympy.Symbol("j")

    order = 4
    param = 1
    expr = 0
    for n in range(0, order + 1):
        for m in range(0, order + 1):
            if n + m > order:
                continue
            e = sympy.Symbol("p" + str(param))
            param += 1
            if n == 1:
                e = e * rpm
            elif n > 1:
                e = e * rpm ** n
            if m == 1:
                e = e * j
            elif m > 1:
                e = e * j ** m
            expr = expr + e

    sub, err = func_approx.approximate(
        func=expr,
        input_data={"rpm": rpms, "j": js},
        output_data=cts)
    ct_expr = expr.subs(sub)
    print("Ct approximation error:", err)

    sub, err = func_approx.approximate(
        func=expr,
        input_data={"rpm": rpms, "j": js},
        output_data=cps)
    cp_expr = expr.subs(sub)
    print("Cp approximation error:", err)

    return ct_expr, cp_expr


def plot_table_data(table: List[Entry],
                    ct_expr: Optional[sympy.Symbol],
                    cp_expr: Optional[sympy.Symbol]):
    rpms = numpy.array([entry.rpm for entry in table])
    js = numpy.array([entry.j for entry in table])
    cts = numpy.array([entry.ct for entry in table])
    cps = numpy.array([entry.cp for entry in table])

    rpms2 = sorted(list(set(rpms)))
    js2 = sorted(list(set(js)))
    rpms2, js2 = numpy.meshgrid(rpms2, js2)

    fig = plt.figure()

    ax0 = fig.add_subplot(1, 2, 1, projection='3d')
    ax0.scatter(rpms, js, cts, color='red')
    ax0.set_xlabel('RPM')
    ax0.set_ylabel('J')
    ax0.set_title("Ct")

    if ct_expr is not None:
        cts2 = func_approx.evaluate(ct_expr, {"rpm": rpms2, "j": js2})
        ax0.plot_surface(rpms2, js2, cts2, color='blue', alpha=0.5)

        cts2 = func_approx.evaluate(ct_expr, {"rpm": rpms, "j": js})
        ax0.scatter(rpms, js, cts2, color='blue')

    ax1 = fig.add_subplot(1, 2, 2, projection='3d')
    ax1.scatter(rpms, js, cps, color='red')
    ax1.set_xlabel('RPM')
    ax1.set_ylabel('J')
    ax1.set_title("Cp")

    if cp_expr is not None:
        cps2 = func_approx.evaluate(cp_expr, {"rpm": rpms2, "j": js2})
        ax1.plot_surface(rpms2, js2, cps2, color='blue', alpha=0.5)

        cps2 = func_approx.evaluate(cp_expr, {"rpm": rpms, "j": js})
        ax1.scatter(rpms, js, cps2, color='blue')

    plt.show()


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('files', type=str,  nargs='+', metavar='FILE',
                        help="propeller files to read")
    args = parser.parse_args(args)

    for file in args.files:
        table = read_propeller_table(file)
        interpolate_table(table)
        ct_expr, cp_expr = approximate_table(table)
        plot_table_data(table, ct_expr, cp_expr)
        break


if __name__ == '__main__':
    run()
