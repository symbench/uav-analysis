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

from typing import Set, Dict, List, Callable, Optional, Generator

import math
import numpy
import sympy
import scipy.optimize


def parameters(prefix: str = "param") -> Generator[sympy.Symbol, None, None]:
    idx = 0
    while True:
        idx += 1
        yield sympy.Symbol(prefix + str(idx))


def get_symbols(expr: sympy.Expr) -> Set[str]:
    """
    Returns all symbols of this expression in a set.
    """
    symbols = set()

    def traverse(e: sympy.Expr):
        if isinstance(e, float) or isinstance(e, int):
            return
        if e.func == sympy.Symbol:
            symbols.add(e.name)
        for a in e.args:
            traverse(a)

    traverse(expr)
    return symbols


def evaluate(expr: sympy.Expr, input_data: Dict[str, numpy.ndarray]) -> numpy.ndarray:
    """
    Evaluates the given expression with the input data and returns the output.
    All numpy array must be of the same size or at least broadcastable and the
    output shape will be the broadcasted shape.
    """
    if (isinstance(expr, float) or isinstance(expr, int)
            or expr.func == sympy.Float
            or expr.func == sympy.Integer
            or expr.func == sympy.core.numbers.Rational
            or expr.func == sympy.core.numbers.NegativeOne
            or expr.func == sympy.core.numbers.Zero
            or expr.func == sympy.core.numbers.One
            or expr.func == sympy.core.numbers.Pi
            or expr.func == sympy.core.numbers.Half):
        return numpy.full((), float(expr))
    elif isinstance(expr, sympy.core.numbers.NaN):
        raise ValueError("NaN value encountered, maybe from dividing by zero")
    elif expr.func == sympy.Symbol:
        return input_data[expr.name]
    elif expr.func == sympy.Add:
        value = evaluate(expr.args[0], input_data)
        for arg in expr.args[1:]:
            value = value + evaluate(arg, input_data)
        return value
    elif expr.func == sympy.Mul:
        value = evaluate(expr.args[0], input_data)
        for arg in expr.args[1:]:
            value = value * evaluate(arg, input_data)
        return value
    elif expr.func == sympy.Pow:
        assert len(expr.args) == 2
        value0 = evaluate(expr.args[0], input_data)
        value1 = float(expr.args[1])
        return numpy.power(value0, value1)
    else:
        raise ValueError(
            "Unknown symbolic expression " + str(type(expr)))


def approximate(func: sympy.Expr, input_data: Dict[str, numpy.ndarray],
                output_data: numpy.ndarray) -> Dict[str, float]:
    """
    Takes a symbolic expression, and input dataset and and output dataset.
    All dataset arrays must be of the same shape or at least broadcastable.
    The symbolic expressions has bound variables by the input dataset and
    free parameters that we want to optimize. Returns an assignment of
    values to free parameters that minimizes the square error of the
    expected and approximated output data.
    """

    if False:
        for key, val in input_data.items():
            print("Input", key, "shape:", val.shape)
        print("Output shape:", output_data.shape)

    symbols = get_symbols(func)
    param_vars = list(symbols - set(input_data.keys()))
    assert len(param_vars) >= 1

    # common shape
    shape = numpy.broadcast(*input_data.values(), output_data).shape

    class Function(Callable):
        def __call__(self, params: List[float]):
            assert len(params) == len(param_vars)
            func2 = func.subs({var: params[idx]
                               for idx, var in enumerate(param_vars)})
            output_data2 = evaluate(func2, input_data)
            result = output_data2 - output_data
            assert result.shape == shape
            return result.flatten()

    class Jacobian(Callable):
        def __init__(self):
            self.diffs = [func.diff(var) for var in param_vars]

        def __call__(self, params: List[float]):
            assert len(params) == len(param_vars)
            subs = {var: params[idx] for idx, var in enumerate(param_vars)}
            result = numpy.empty(shape=(numpy.prod(shape), len(param_vars)))
            for idx, diff in enumerate(self.diffs):
                diff = evaluate(diff.subs(subs), input_data)
                result[:, idx] = numpy.broadcast_to(diff, shape).flatten()
            return result

    init = numpy.zeros(shape=(len(param_vars),))
    result = scipy.optimize.least_squares(
        fun=Function(),
        x0=init,
        jac=Jacobian(),
    )

    if not result.success:
        print("WARNING: apprixmation failed with cost", result.cost)

    params = {var: result.x[idx] for idx, var in enumerate(param_vars)}
    return params, result.cost


def linear_approx(func: sympy.Expr, input_data: Dict[str, numpy.ndarray],
                  output_data: numpy.ndarray,
                  rcond: Optional[float] = 1e-20) -> Dict[str, float]:
    """
    Takes a symbolic expression, and input dataset and and output dataset.
    All dataset arrays must be of the same shape or at least broadcastable.
    The symbolic expressions has bound variables by the input dataset and
    free parameters that we want to optimize. Returns an assignment of
    values to free parameters that minimizes the square error of the
    expected and approximated output data. The rcond parameter controls the
    cutoff ratio for the small singular values.
    """

    if False:
        for key, val in input_data.items():
            print("Input", key, "shape:", val.shape)
        print("Output shape:", output_data.shape)

    symbols = get_symbols(func)
    param_vars = list(symbols - set(input_data.keys()))
    assert len(param_vars) >= 1

    # common shape
    shape = numpy.broadcast(*input_data.values(), output_data).shape

    matrix = numpy.empty(shape=(numpy.prod(shape), len(param_vars)))
    for idx, var in enumerate(param_vars):
        diff = func.diff(var)
        diff = evaluate(diff, input_data)
        diff = numpy.broadcast_to(diff, shape)
        matrix[:, idx] = diff.flatten()

    output_data = numpy.broadcast_to(output_data, shape).flatten()
    result, cost, rank, sings = numpy.linalg.lstsq(
        matrix, output_data, rcond=rcond)

    if rank < len(param_vars):
        print("Matrix shape:", matrix.shape)
        print("Matrix rank:", rank)
        print("Singular values:", sings)
        raise ValueError("Singluar problem")

    assert len(cost) == 1
    return {var: result[idx] for idx, var in enumerate(param_vars)}, cost[0]


def approx_error(func: sympy.Expr, input_data: Dict[str, numpy.ndarray],
                 output_data: numpy.ndarray) -> float:
    data = evaluate(func, input_data) - output_data
    return math.sqrt(numpy.mean(numpy.square(data)))


if __name__ == '__main__':
    a = sympy.Symbol('a')
    b = sympy.Symbol('b')
    c = sympy.Symbol('c')
    d = sympy.Symbol('d')
    x = sympy.Symbol('x')
    y = sympy.Symbol('y')
    expr = a * x + b * y + c * y ** 3 + d

    input_data = {
        'x': numpy.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        'y': numpy.array([[1.0, 0.0, 2.0], [0.0, 1.0, 0.0]]),
    }
    output_data = numpy.array([[3.0, 5.0, 13.0], [9.0, 11.0, 13.0]])

    print("Expression:", expr)
    print("Input:")
    print(input_data)
    print("Output:")
    print(output_data)

    subs, err = linear_approx(expr, input_data, output_data)
    print("Substitution:", subs)
    print("Error:", err)

    expr = expr.subs(subs)
    print("Expression:", expr)
    print("Approximate output:")
    print(evaluate(expr, input_data))
