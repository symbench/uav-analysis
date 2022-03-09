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

from typing import List, Dict, Any

import math
import numpy
import sympy

from uav_analysis.testbench_data import TestbenchData
from uav_analysis.func_approx import approximate


def inertia_matrix(ixx, ixy, ixz, iyy, iyz, izz) -> numpy.ndarray:
    """
    Takes 6 values or arrays of shape [*] and returns the inertia tensor
    matrix of shape [*, 3, 3].
    """
    return numpy.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def translate_inertia(
        inertia: numpy.ndarray,
        mass: numpy.ndarray,
        offset: numpy.ndarray) -> numpy.ndarray:
    """
    Returns the moment of inertia matrix with respect to a different point from the
    moment of inertia through the center of mass point. The inertia matrix must be
    of shape [*, 3, 3], the mass must be of shape [*], and the offset be of shape
    [*, 3], the returned matrix is of shape [*, 3, 3]. If you would like to transfer
    the moment of inertia back to the center of mass, then use negative mass value.
    """
    mat = numpy.array([
        [offset[1] ** 2 + offset[2] ** 2, -offset[0]
            * offset[1], -offset[0] * offset[2]],
        [-offset[0] * offset[1], offset[0] ** 2 +
            offset[2] ** 2, -offset[1] * offset[2]],
        [-offset[0] * offset[2], -offset[1] *
            offset[2], offset[0] ** 2 + offset[1] ** 2]
    ])
    return inertia + mass * mat


def quad_copter_fixed_bemp(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
    ])

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 + C() * L1)

    x_cm = fit('aircraft.x_cm', C() / mass)
    y_cm = fit('aircraft.y_cm', C() / mass)
    z_cm = fit('aircraft.z_cm', (C() + C() * L0 +
                                 C() * L1 + C() * L1 ** 2) / mass)

    # we compensate for the center of gravity offset
    fit('aircraft.Ixx', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        - mass * (y_cm ** 2 + z_cm ** 2))
    fit('aircraft.Iyy', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        - mass * (x_cm ** 2 + z_cm ** 2))
    fit('aircraft.Izz', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        - mass * (x_cm ** 2 + y_cm ** 2))
    fit('aircraft.Ixy', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        + mass * x_cm * y_cm)
    fit('aircraft.Ixz', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        + mass * x_cm * z_cm)
    fit('aircraft.Iyz', C() + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
        + mass * y_cm * z_cm)

    fit('aircraft.X_fuseuu', C() + C() * L0 + C() * L1)
    fit('aircraft.Y_fusevv', C() + C() * L0 + C() * L1)
    fit('aircraft.Z_fuseww', C() + C() * L0 + C() * L1)

    fit('Prop_0_x', C() * L0)
    fit('Prop_0_y', C() * L0)
    fit('Prop_0_z', C())
    fit('Prop_1_x', C() * L0)
    fit('Prop_1_y', C() * L0)
    fit('Prop_1_z', C())
    fit('Prop_2_x', C() * L0)
    fit('Prop_2_y', C() * L0)
    fit('Prop_2_z', C())
    fit('Prop_3_x', C() * L0)
    fit('Prop_3_y', C() * L0)
    fit('Prop_3_z', C())

    return result


def quad_copter_fixed_bemp2(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg
    L8 = sympy.Symbol('Length_8')  # battery offset
    L9 = sympy.Symbol('Length_9')  # battery offset

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
        'Length_8',
        'Length_9',
    ])

    for var in input_data.keys():
        print(var, "min:", min(input_data[var]), "max:", max(input_data[var]))

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 + C() * L1)

    x_cm = fit('aircraft.x_cm', (C() + C() * L8 + C() * L9) / mass)
    y_cm = fit('aircraft.y_cm', (C() + C() * L8 + C() * L9) / mass)
    z_cm = fit('aircraft.z_cm', (C() + C() * L0 +
                                 C() * L1 + C() * L1 ** 2) / mass)

    if True:
        # we compensate for the center of gravity offset
        fit('aircraft.Ixx', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (y_cm ** 2 + z_cm ** 2))
        fit('aircraft.Iyy', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (x_cm ** 2 + z_cm ** 2))
        fit('aircraft.Izz', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (x_cm ** 2 + y_cm ** 2))
        fit('aircraft.Ixy', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * x_cm * y_cm)
        fit('aircraft.Ixz', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * x_cm * z_cm)
        fit('aircraft.Iyz', C() + C() * L0 + C() * L1
            + C() * L0 ** 2 + C() * L1 ** 2
            + C() * L0 ** 3 + C() * L0 ** 2 * L1 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * y_cm * z_cm)

    fit('aircraft.X_fuseuu', C()
        + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L1 ** 3)
    fit('aircraft.Y_fusevv', C()
        + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L1 ** 3)
    fit('aircraft.Z_fuseww', C()
        + C() * L0 + C() * L1
        + C() * L0 ** 2 + C() * L1 ** 2
        + C() * L0 ** 3 + C() * L1 ** 3)

    fit('Prop_0_x', C() * L0)
    fit('Prop_0_y', C() * L0)
    fit('Prop_0_z', C())
    fit('Prop_1_x', C() * L0)
    fit('Prop_1_y', C() * L0)
    fit('Prop_1_z', C())
    fit('Prop_2_x', C() * L0)
    fit('Prop_2_y', C() * L0)
    fit('Prop_2_z', C())
    fit('Prop_3_x', C() * L0)
    fit('Prop_3_y', C() * L0)
    fit('Prop_3_z', C())

    return result


def hplane_fixed_bemp(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L1 = sympy.Symbol('Length_1')  # arm length
    L8 = sympy.Symbol('Length_8')  # battery offset
    L9 = sympy.Symbol('Length_9')  # battery offset

    input_data = data.get_tables([
        'Length_1',
        'Length_8',
        'Length_9',
    ])

    for var in input_data.keys():
        print(var, "min:", min(input_data[var]), "max:", max(input_data[var]))

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L1)

    x_cm = fit('aircraft.x_cm', (C() + C() * L1 + C() * L8 + C() * L9) / mass)
    y_cm = fit('aircraft.y_cm', (C() + C() * L8 + C() * L9) / mass)
    z_cm = fit('aircraft.z_cm', (C() + C() * L1) / mass)

    if True:
        # we compensate for the center of gravity offset
        fit('aircraft.Ixx', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (y_cm ** 2 + z_cm ** 2))
        fit('aircraft.Iyy', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (x_cm ** 2 + z_cm ** 2))
        fit('aircraft.Izz', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            - mass * (x_cm ** 2 + y_cm ** 2))
        fit('aircraft.Ixy', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * x_cm * y_cm)
        fit('aircraft.Ixz', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * x_cm * z_cm)
        fit('aircraft.Iyz', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3
            + C() * L8 + C() * L9 + C() * L8 ** 2 + C() * L9 ** 2 + C() * L8 * L9
            + mass * y_cm * z_cm)

    fit('aircraft.X_fuseuu', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3)
    fit('aircraft.Y_fusevv', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3)
    fit('aircraft.Z_fuseww', C() + C() * L1 + C() * L1 ** 2 + C() * L1 ** 3)

    fit('Rear_Prop_L_x', C() - L1)
    fit('Rear_Prop_L_y', C())
    fit('Rear_Prop_L_z', C())
    fit('Rear_Prop_R_x', C() - L1)
    fit('Rear_Prop_R_y', C())
    fit('Rear_Prop_R_z', C())
    fit('Front_Prop_L_x', C() + L1)
    fit('Front_Prop_L_y', C())
    fit('Front_Prop_L_z', C())
    fit('Front_Prop_C_x', C() + L1)
    fit('Front_Prop_C_y', C())
    fit('Front_Prop_C_z', C())
    fit('Front_Prop_R_x', C() + L1)
    fit('Front_Prop_R_y', C())
    fit('Front_Prop_R_z', C())
    fit('Left_Wing_x', C())
    fit('Left_Wing_y', C())
    fit('Left_Wing_z', C())
    fit('Right_Wing_x', C())
    fit('Right_Wing_y', C())
    fit('Right_Wing_z', C())

    return result


def quad_copter_batt(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg
    L8 = sympy.Symbol('Length_8')  # battery position x
    L9 = sympy.Symbol('Length_9')  # battery position y
    B0 = sympy.Symbol('Battery_0_Weight')
    B1 = sympy.Symbol('Battery_0_Length')
    B2 = sympy.Symbol('Battery_0_Width')
    B3 = sympy.Symbol('Battery_0_Thickness')

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
        'Length_8',
        'Length_9',
        'Battery_0_Weight',
        'Battery_0_Length',
        'Battery_0_Width',
        'Battery_0_Thickness',
    ])

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    def powers(vars: List[Any], exp: int):
        assert exp >= 1
        result = [[]]
        for _ in range(exp):
            result2 = []
            for elem1 in result:
                next = max(elem1) if elem1 else 0
                for elem2 in range(next, len(vars)):
                    result2.append(elem1 + [elem2])
            result = result2

        sum = 0
        for elem1 in result:
            value = C()
            for idx in elem1:
                value = value * vars[idx]
            sum = sum + value

        return sum

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 + C() * L1 + B0)

    x_cm = fit('aircraft.x_cm',
               (C() + C() * B0 + C() * B0 * L8 + C() * B0 * L9) / mass)
    y_cm = fit('aircraft.y_cm',
               (C() + C() * B0 + C() * B0 * L8 + C() * B0 * L9) / mass)
    z_cm = fit('aircraft.z_cm',
               (C() + C() * L0 + C() * L1 + C() * L1 ** 2
                   + C() * B0 + C() * B0 * B2) / mass)

    # we compensate for the center of gravity offset
    if True:
        fit('aircraft.Ixx',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            - mass * (y_cm ** 2 + z_cm ** 2))

        fit('aircraft.Iyy',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            - mass * (x_cm ** 2 + z_cm ** 2))

        fit('aircraft.Izz',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            - mass * (x_cm ** 2 + y_cm ** 2))

        fit('aircraft.Ixy',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + mass * x_cm * y_cm)

        fit('aircraft.Ixz',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + mass * x_cm * z_cm)

        fit('aircraft.Iyz',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + mass * y_cm * z_cm)

    if True:
        fit('aircraft.X_fuseuu',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2))

        fit('aircraft.Y_fusevv',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2))

        fit('aircraft.Z_fuseww',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2))

    if True:
        fit('Prop_0_x', C() * L0)
        fit('Prop_0_y', C() * L0)
        fit('Prop_0_z', C())
        fit('Prop_1_x', C() * L0)
        fit('Prop_1_y', C() * L0)
        fit('Prop_1_z', C())
        fit('Prop_2_x', C() * L0)
        fit('Prop_2_y', C() * L0)
        fit('Prop_2_z', C())
        fit('Prop_3_x', C() * L0)
        fit('Prop_3_y', C() * L0)
        fit('Prop_3_z', C())

    return result


def quad_copter_prop(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg
    L8 = sympy.Symbol('Length_8')  # battery position x
    L9 = sympy.Symbol('Length_9')  # battery position y
    P0 = sympy.Symbol('Prop_0_Weight')
    P1 = sympy.Symbol('Prop_0_Diameter')
    P2 = sympy.Symbol('Prop_0_Thickness')

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
        'Length_8',
        'Length_9',
        'Prop_0_Weight',
        'Prop_0_Diameter',
        'Prop_0_Thickness',
    ])

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    def powers(vars: List[Any], exp: int):
        assert exp >= 1
        result = [[]]
        for _ in range(exp):
            result2 = []
            for elem1 in result:
                next = max(elem1) if elem1 else 0
                for elem2 in range(next, len(vars)):
                    result2.append(elem1 + [elem2])
            result = result2

        sum = 0
        for elem1 in result:
            value = C()
            for idx in elem1:
                value = value * vars[idx]
            sum = sum + value

        return sum

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 + C() * L1 + 4 * P0)

    x_cm = fit('aircraft.x_cm',
               (C() + C() * L8 + C() * L9) / mass)
    y_cm = fit('aircraft.y_cm',
               (C() + C() * L8 + C() * L9) / mass)
    z_cm = fit('aircraft.z_cm',
               (C() + C() * L0 + C() * L1 + C() * L1 ** 2 + C() * P0) / mass)

    # we compensate for the center of gravity offset
    if True:
        fit('aircraft.Ixx',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (y_cm ** 2 + z_cm ** 2))

        fit('aircraft.Iyy',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (x_cm ** 2 + z_cm ** 2))

        fit('aircraft.Izz',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (x_cm ** 2 + y_cm ** 2))

        fit('aircraft.Ixy',
            C()
            + powers([L8, L9], 1)
            + powers([L8, L9], 2)
            + mass * x_cm * y_cm)

        fit('aircraft.Ixz',
            C()
            + powers([L0, L1, L8, L9], 1)
            + powers([L0, L1, L8, L9], 2)
            + P0 * powers([P1, P2, L0], 2)
            + mass * x_cm * z_cm)

        fit('aircraft.Iyz',
            C()
            + powers([L0, L1, L8, L9], 1)
            + powers([L0, L1, L8, L9], 2)
            + P0 * powers([P1, P2, L0], 2)
            + mass * y_cm * z_cm)

    if True:
        fit('aircraft.X_fuseuu',
            C()
            + powers([L0, L1], 1)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

        fit('aircraft.Y_fusevv',
            C()
            + powers([L0, L1], 1)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

        fit('aircraft.Z_fuseww',
            C()
            + powers([L0, L1], 1)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

    if True:
        fit('Prop_0_x', C() * L0)
        fit('Prop_0_y', C() * L0)
        fit('Prop_0_z', C())
        fit('Prop_1_x', C() * L0)
        fit('Prop_1_y', C() * L0)
        fit('Prop_1_z', C())
        fit('Prop_2_x', C() * L0)
        fit('Prop_2_y', C() * L0)
        fit('Prop_2_z', C())
        fit('Prop_3_x', C() * L0)
        fit('Prop_3_y', C() * L0)
        fit('Prop_3_z', C())

    return result


def quad_copter_batt_prop(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg
    L8 = sympy.Symbol('Length_8')  # battery position x
    L9 = sympy.Symbol('Length_9')  # battery position y
    B0 = sympy.Symbol('Battery_0_Weight')
    B1 = sympy.Symbol('Battery_0_Length')
    B2 = sympy.Symbol('Battery_0_Width')
    B3 = sympy.Symbol('Battery_0_Thickness')
    P0 = sympy.Symbol('Prop_0_Weight')
    P1 = sympy.Symbol('Prop_0_Diameter')
    P2 = sympy.Symbol('Prop_0_Thickness')

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
        'Length_8',
        'Length_9',
        'Battery_0_Weight',
        'Battery_0_Length',
        'Battery_0_Width',
        'Battery_0_Thickness',
        'Prop_0_Weight',
        'Prop_0_Diameter',
        'Prop_0_Thickness',
    ])

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    def powers(vars: List[Any], exp: int):
        assert exp >= 1
        result = [[]]
        for _ in range(exp):
            result2 = []
            for elem1 in result:
                next = max(elem1) if elem1 else 0
                for elem2 in range(next, len(vars)):
                    result2.append(elem1 + [elem2])
            result = result2

        sum = 0
        for elem1 in result:
            value = C()
            for idx in elem1:
                value = value * vars[idx]
            sum = sum + value

        return sum

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 + C() * L1 + B0 + 4 * P0)

    x_cm = fit('aircraft.x_cm',
               (C() + C() * B0 + C() * B0 * L8 + C() * B0 * L9) / mass)
    y_cm = fit('aircraft.y_cm',
               (C() + C() * B0 + C() * B0 * L8 + C() * B0 * L9) / mass)
    z_cm = fit('aircraft.z_cm',
               (C() + C() * L0 + C() * L1 + C() * L1 ** 2
                + C() * B0 + C() * B0 * B2
                + C() * P0 + C() * P0 * P2) / mass)

    if True:
        # we compensate for the center of gravity offset
        fit('aircraft.Ixx',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (y_cm ** 2 + z_cm ** 2))

        fit('aircraft.Iyy',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (x_cm ** 2 + z_cm ** 2))

        fit('aircraft.Izz',
            C()
            + powers([L0, L1], 1)
            + powers([L0, L1], 2)
            + powers([L0, L1], 3)
            + powers([L8, L9], 2)
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            - mass * (x_cm ** 2 + y_cm ** 2))

        fit('aircraft.Ixy',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            + mass * x_cm * y_cm)

        fit('aircraft.Ixz',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            + mass * x_cm * z_cm)

        fit('aircraft.Iyz',
            C()
            + B0 * C()
            + B0 * powers([B2, B3, L8, L9], 1)
            + B0 * powers([B2, B3, L8, L9], 2)
            + P0 * powers([L0, P1, P2], 2)
            + P0 * powers([L0, P1, P2], 3)
            + mass * y_cm * z_cm)

    if True:
        fit('aircraft.X_fuseuu',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

        fit('aircraft.Y_fusevv',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

        fit('aircraft.Z_fuseww',
            C()
            + powers([L0, L1], 1)
            + powers([B1, B2, B3], 1)
            + powers([B1, B2, B3], 2)
            + powers([P1, P2], 1)
            + powers([P1, P2], 2))

    if True:
        fit('Prop_0_x', C() * L0)
        fit('Prop_0_y', C() * L0)
        fit('Prop_0_z', C())
        fit('Prop_1_x', C() * L0)
        fit('Prop_1_y', C() * L0)
        fit('Prop_1_z', C())
        fit('Prop_2_x', C() * L0)
        fit('Prop_2_y', C() * L0)
        fit('Prop_2_z', C())
        fit('Prop_3_x', C() * L0)
        fit('Prop_3_y', C() * L0)
        fit('Prop_3_z', C())

    return result


def quad_copter_batt_prop_motor(data: 'TestbenchData') -> Dict[str, sympy.Expr]:
    L0 = sympy.Symbol('Length_0')  # arm length
    L1 = sympy.Symbol('Length_1')  # support leg
    B0 = sympy.Symbol('Battery_0_Weight')
    B1 = sympy.Symbol('Battery_0_Length')
    B2 = sympy.Symbol('Battery_0_Width')
    B3 = sympy.Symbol('Battery_0_Thickness')
    P0 = sympy.Symbol('Prop_0_Weight')
    P1 = sympy.Symbol('Prop_0_Diameter')
    P2 = sympy.Symbol('Prop_0_Thickness')
    M0 = sympy.Symbol('Motor_0_Weight')
    M1 = sympy.Symbol('Motor_0_TotalLength')
    M2 = sympy.Symbol('Motor_0_Length')
    M3 = sympy.Symbol('Motor_0_CanLength')

    input_data = data.get_tables([
        'Length_0',
        'Length_1',
        'Battery_0_Weight',
        'Battery_0_Length',
        'Battery_0_Width',
        'Battery_0_Thickness',
        'Prop_0_Weight',
        'Prop_0_Diameter',
        'Prop_0_Thickness',
        'Motor_0_Weight',
        'Motor_0_TotalLength',
        'Motor_0_Length',
        'Motor_0_CanLength',
    ])

    param = 0

    def C():
        nonlocal param
        param += 1
        return sympy.Symbol("param_" + str(param))

    result = dict()

    def fit(name, expr):
        if data.has_field(name):
            subs, error = approximate(expr, input_data, data.get_table(name))
            print("INFO:", name, "approx error", error)
            result[name] = expr.subs(subs)
        else:
            result[name] = 0.0
            print("WARNING: missing data for", name)
        return result[name]

    mass = fit('aircraft.mass', C() + C() * L0 +
               C() * L1 + B0 + 4 * P0 + 4 * M0)

    x_cm = fit('aircraft.x_cm', (
        C()
        + C() * B0 + C() * B0 * B1 + C() * B0 * B3
    ) / mass)
    y_cm = fit('aircraft.y_cm', (
        C()
        + C() * B0 + C() * B0 * B3 + C() * B0 * B3
    ) / mass)
    z_cm = fit('aircraft.z_cm', (
        C()
        + L0 * C()
        + L1 * (C() + C() * L1)
        + B0 * (C() + C() * B2)
        + M0 * (C() + C() * M1 + C() * M2)
        + P0 * (C() + C() * P1 + C() * P2 + C() * M1 + C() * M2 + C() * M3)
    ) / mass)

    if False:
        fit('aircraft.X_fuseuu', C() + C() * L0 + C() * L1
            + C() * B1 * B2 + C() * B1 * B3 + C() * B2 * B3
            + C() * P1 + C() * P2 + C() * P1 * P2 + C() * P1 * P1)
        fit('aircraft.Y_fusevv', C() + C() * L0 + C() * L1
            + C() * B1 * B2 + C() * B1 * B3 + C() * B2 * B3
            + C() * P1 + C() * P2 + C() * P1 * P2 + C() * P1 * P1)
        fit('aircraft.Z_fuseww', C() + C() * L0 + C() * L1
            + C() * B1 * B2 + C() * B1 * B3 + C() * B2 * B3
            + C() * P1 + C() * P2 + C() * P1 * P2 + C() * P1 * P1)

    if False:
        fit('Prop_0_x', C() * L0)
        fit('Prop_0_y', C() * L0)
        fit('Prop_0_z', C())
        fit('Prop_1_x', C() * L0)
        fit('Prop_1_y', C() * L0)
        fit('Prop_1_z', C())
        fit('Prop_2_x', C() * L0)
        fit('Prop_2_y', C() * L0)
        fit('Prop_2_z', C())
        fit('Prop_3_x', C() * L0)
        fit('Prop_3_y', C() * L0)
        fit('Prop_3_z', C())

    return result


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('file', type=str,  nargs="+", metavar='FILE',
                        help='a zip log files to read')
    parser.add_argument('--model',
                        default='quad-copter-fixed-bemp',
                        choices=[
                            'quad-copter-fixed-bemp',
                            'quad-copter-fixed-bemp2',
                            'quad-copter-batt',
                            'quad-copter-prop',
                            'quad-copter-batt-prop',
                            'quad-copter-batt-prop-motor',
                            'hplane-fixed-bemp',
                        ],
                        help='selects the analysis model')
    args = parser.parse_args(args)

    data = TestbenchData()
    for file in args.file:
        print("Reading", file)
        data.load(file)

    print("Number of valid runs:", len(data.get_table('Length_0')))
    if len(data.get_table('Length_0')) == 0:
        return

    print("Using model:", args.model)
    if args.model == 'quad-copter-fixed-bemp':
        formulas = quad_copter_fixed_bemp(data)
    elif args.model == 'quad-copter-fixed-bemp2':
        formulas = quad_copter_fixed_bemp2(data)
    elif args.model == 'quad-copter-batt':
        formulas = quad_copter_batt(data)
    elif args.model == 'quad-copter-prop':
        formulas = quad_copter_prop(data)
    elif args.model == 'quad-copter-batt-prop':
        formulas = quad_copter_batt_prop(data)
    elif args.model == 'quad-copter-batt-prop-motor':
        formulas = quad_copter_batt_prop_motor(data)
    elif args.model == 'hplane-fixed-bemp':
        formulas = hplane_fixed_bemp(data)
    else:
        print("Unknown analyis model:", args.model)
        return

    for key, val in formulas.items():
        print(key, "=", val)


if __name__ == '__main__':
    run()
