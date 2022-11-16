#!/usr/bin/env python3
# Copyright (C) 2021-2022, Miklos Maroti
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

from typing import Any, Dict, Optional, Tuple

import json
import numpy
import os
import sympy

from .components import BATTERIES, AERO_INFO, DATA_PATH
from .wing_analysis import get_wing_thickness, get_wing_data, get_wing_weight

from constraint_prog.point_cloud import PointCloud, PointFunc
from constraint_prog.sympy_func import pareto_func


AIR_DENSITY = 1.225  # kg/m^3
GRAVITATION = 9.81   # m/s^2


class SimpleBatteryModel():
    def __init__(self,
                 battery_name: str,
                 series_count: Optional[int] = None,
                 parallel_count: Optional[int] = None,
                 prefix: str = "battery"):

        battery = BATTERIES[battery_name]
        self.battery_name = battery_name
        self.prefix = prefix

        single_voltage = float(battery["VOLTAGE"])           # V
        single_capacity = float(battery["CAPACITY"]) * 1e-3  # Ah
        single_weight = float(battery["WEIGHT"])             # kg
        single_volume = float(battery["LENGTH"]) \
            * float(battery["WIDTH"]) \
            * float(battery["THICKNESS"]) * 1e-9                # m^3
        discharge_rate = float(battery["CONT_DISCHARGE_RATE"])  # C

        if series_count is None:
            series_count = sympy.Symbol(self.prefix + "_series_count")
        self.series_count = series_count

        if parallel_count is None:
            parallel_count = sympy.Symbol(self.prefix + "_parallel_count")
        self.parallel_count = parallel_count

        self.voltage = single_voltage * self.series_count
        self.capacity = single_capacity * self.parallel_count
        self.weight = single_weight * self.series_count * self.parallel_count
        self.volume = single_volume * self.series_count * self.parallel_count
        self.current = self.capacity * discharge_rate
        self.energy = self.voltage * self.capacity

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()
        if isinstance(self.series_count, sympy.Symbol):
            bounds[self.series_count.name] = (1.0, 100.0)
        if isinstance(self.parallel_count, sympy.Symbol):
            bounds[self.parallel_count.name] = (1.0, 50.0)
        return bounds

    def report(self) -> Dict[str, Any]:
        report = {
            self.prefix + "_series_count": self.series_count,
            self.prefix + "_parallel_count": self.parallel_count,
            self.prefix + "_voltage": self.voltage,
            self.prefix + "_current": self.current,
            self.prefix + "_capacity": self.capacity,
            self.prefix + "_energy": self.energy,
            self.prefix + "_weight": self.weight,
            self.prefix + "_volume": self.volume,
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report

    def constraints(self):
        return {}


class ParetoBatteryModel():
    PARETO_FUNC = None

    @staticmethod
    def create_pareto_func():
        cloud = PointCloud.load(
            os.path.join(DATA_PATH, "battery_analysis_pareto.csv"),
            silent=True,
        )
        cloud = cloud.projection([
            "total_voltage",
            "total_capacity",
            "total_current",
            "total_weight",
        ])
        ParetoBatteryModel.PARETO_FUNC = pareto_func(
            "battery_analysis_pareto",
            cloud,
            directions=[
                +1.0,
                +1.0,
                +1.0,
                -1.0,
            ])

    def __init__(self,
                 max_voltage: float = 100.0,    # V
                 max_capacity: float = 1000.0,  # Ah
                 max_current: float = 1000.0,   # A
                 max_weight: float = 5.0,       # kg
                 prefix: str = "battery"):
        if ParetoBatteryModel.PARETO_FUNC is None:
            ParetoBatteryModel.create_pareto_func()

        self.max_voltage = max_voltage
        self.max_capacity = max_capacity
        self.max_current = max_current
        self.max_weight = max_weight
        self.prefix = prefix

        self.voltage = sympy.Symbol(self.prefix + "_voltage")
        self.capacity = sympy.Symbol(self.prefix + "_capacity")
        self.current = sympy.Symbol(self.prefix + "_current")
        self.weight = sympy.Symbol(self.prefix + "_weight")

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        return {
            self.prefix + "_voltage": (0.0, self.max_voltage),
            self.prefix + "_capacity": (0.0, self.max_capacity),
            self.prefix + "_current": (0.0, self.max_current),
            self.prefix + "_weight": (0.0, self.max_weight),
        }

    def report(self) -> Dict[str, Any]:
        return {}

    def constraints(self):
        return {
            self.prefix + "_pareto": ParetoBatteryModel.PARETO_FUNC(
                self.voltage, self.capacity, self.current, self.weight)
        }


class WingModel():
    def __init__(self,
                 naca_profile: str,
                 max_load: Optional[float] = None,  # N
                 chord: Optional[float] = None,     # m
                 span: Optional[float] = None,      # m
                 prefix: str = "wing"):
        self.naca_profile = naca_profile
        self.thickness = get_wing_thickness(naca_profile)
        self.prefix = prefix

        if max_load is None:
            max_load = sympy.Symbol(self.prefix + "_max_load")
        self.max_load = max_load

        if chord is None:
            chord = sympy.Symbol(self.prefix + "_chord")
        self.chord = chord

        if span is None:
            span = sympy.Symbol(self.prefix + "_span")
        self.span = span

        self.weight = get_wing_weight(
            thickness=self.thickness,
            chord1=self.chord,
            chord2=self.chord,
            span=self.span,
            max_load=self.max_load)

        self.wing_data = get_wing_data(
            naca_profile=naca_profile,
            chord1=self.chord,
            chord2=self.chord,
            span=self.span)

        self.surface_area = self.wing_data["surface_area"]
        self.min_angle = (self.wing_data["C_Lmin"] - self.wing_data["C_L0"]) / \
            self.wing_data["a"]
        self.max_angle = (self.wing_data["C_Lmax"] - self.wing_data["C_L0"]) / \
            self.wing_data["a"]

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()
        if isinstance(self.max_load, sympy.Symbol):
            bounds[self.max_load.name] = (0.0, 500.0)
        if isinstance(self.chord, sympy.Symbol):
            bounds[self.chord.name] = (0.05, 2.0)
        else:
            assert 0.05 <= self.chord <= 2.0
        if isinstance(self.span, sympy.Symbol):
            bounds[self.span.name] = (0.05, 10.0)
        else:
            assert 0.05 <= self.span <= 10.0
        return bounds

    def report(self):
        report = {
            self.prefix + "_thickness": self.thickness,
            self.prefix + "_max_load": self.max_load,
            self.prefix + "_span": self.span,
            self.prefix + "_chord": self.chord,
            self.prefix + "_weight": self.weight,
            self.prefix + "_surface_area": self.surface_area,
            self.prefix + "_min_angle": self.min_angle,
            self.prefix + "_max_angle": self.max_angle,
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report


class LiftModel():
    def __init__(self,
                 wing: WingModel,
                 speed: Optional[float] = None,
                 angle: Optional[float] = None,
                 prefix: str = "liftdrag"):

        self.prefix = prefix

        if speed is None:
            speed = sympy.Symbol(prefix + "_speed")
        self.speed = speed

        if angle is None:
            angle = sympy.Symbol(prefix + "_angle")
        self.angle = angle

        self.min_angle = wing.min_angle
        self.max_angle = wing.max_angle
        assert isinstance(self.min_angle, float)
        assert isinstance(self.max_angle, float)

        C_Lw = wing.wing_data["C_L0"] + wing.wing_data["a"] * self.angle
        C_Dw = wing.wing_data["C_D0"] + wing.wing_data["k"] * C_Lw ** 2

        qbarprime_w = 0.5 * AIR_DENSITY * self.speed ** 2

        self.lift = wing.surface_area * qbarprime_w * C_Lw
        self.drag = wing.surface_area * qbarprime_w * C_Dw

        self.max_load = wing.max_load

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()

        if isinstance(self.speed, sympy.Symbol):
            bounds[self.speed.name] = (1.0, 50.0)
        else:
            assert 0.0 <= self.speed <= 50.0

        if isinstance(self.angle, sympy.Symbol):
            bounds[self.angle.name] = (self.min_angle, self.max_angle)
        else:
            assert self.min_angle <= self.angle <= self.max_angle

        return bounds

    def report(self) -> Dict[str, Any]:
        report = {
            self.prefix + "_speed": self.speed,
            self.prefix + "_angle": self.angle,
            self.prefix + "_lift": self.lift,
            self.prefix + "_drag": self.drag,
            self.prefix + "_load": sympy.sqrt(self.lift ** 2 + self.drag ** 2),
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report

    def constraints(self):
        return {
            "equ_" + self.prefix + "_load": self.lift ** 2 + self.drag ** 2 <= 0.8 * self.max_load ** 2
        }


class MotorPropModel():
    def __init__(self, approx_data: Dict[str, Any]):

        self.motor_name = approx_data["motor_name"]
        self.propeller_name = approx_data["propeller_name"]
        self.weight = approx_data["weight"]

        self.voltage0 = approx_data["min_voltage"]
        self.current0 = approx_data["min_current"]
        self.thrust0 = approx_data["min_thrust"]
        self.current0_at20 = approx_data["min_current_at20"]
        self.thrust0_at20 = approx_data["min_thrust_at20"]

        self.voltage1 = approx_data["med_voltage"]
        self.current1 = approx_data["med_current"]
        self.thrust1 = approx_data["med_thrust"]
        self.current1_at20 = approx_data["med_current_at20"]
        self.thrust1_at20 = approx_data["med_thrust_at20"]

        self.voltage2 = approx_data["max_voltage"]
        self.current2 = approx_data["max_current"]
        self.thrust2 = approx_data["max_thrust"]
        self.current2_at20 = approx_data["max_current_at20"]
        self.thrust2_at20 = approx_data["max_thrust_at20"]

        assert self.voltage0 < self.voltage1 < self.voltage2
        assert self.current0 <= self.current1 <= self.current2
        assert self.thrust0 <= self.thrust1 <= self.thrust2

        self.min_voltage = self.voltage0
        self.max_voltage = self.voltage2

    @staticmethod
    def quadratic_fit(
            x0: float, x1: float, x2: float,
            y0: float, y1: float, y2: float,
            x: Any) -> Any:
        assert x0 < x1 < x2

        a = numpy.array([
            [1, x0, x0 ** 2],
            [1, x1, x1 ** 2],
            [1, x2, x2 ** 2],
        ])
        b = numpy.array([y0, y1, y2])

        c = numpy.linalg.solve(a, b)
        return float(c[0]) + float(c[1]) * x + float(c[2]) * x ** 2

    def get_current(self, voltage: Any, at20: bool = False) -> Any:
        if not at20:
            return MotorPropModel.quadratic_fit(
                self.voltage0, self.voltage1, self.voltage2,
                self.current0, self.current1, self.current2,
                voltage)
        else:
            return MotorPropModel.quadratic_fit(
                self.voltage0, self.voltage1, self.voltage2,
                self.current0_at20, self.current1_at20, self.current2_at20,
                voltage)

    def get_thrust(self, voltage: Any, at20: bool = False) -> Any:
        if not at20:
            return MotorPropModel.quadratic_fit(
                self.voltage0, self.voltage1, self.voltage2,
                self.thrust0, self.thrust1, self.thrust2,
                voltage)
        else:
            return MotorPropModel.quadratic_fit(
                self.voltage0, self.voltage1, self.voltage2,
                self.thrust0_at20, self.thrust1_at20, self.thrust2_at20,
                voltage)


class ThrustModel():
    def __init__(self,
                 motor_prop: MotorPropModel,
                 voltage: Optional[float] = None,
                 min_voltage: Optional[float] = None,
                 max_voltage: Optional[float] = None,
                 at20: bool = False,
                 prefix: str = "thrust"):

        self.at20 = at20
        self.prefix = prefix

        if min_voltage is None:
            min_voltage = motor_prop.min_voltage
        self.min_voltage = min_voltage

        if max_voltage is None:
            max_voltage = motor_prop.max_voltage
        self.max_voltage = max_voltage

        if voltage is None:
            voltage = sympy.Symbol(self.prefix + "_voltage")
        self.voltage = voltage

        self.current = motor_prop.get_current(self.voltage, self.at20)
        self.power = self.voltage * self.current
        self.thrust = motor_prop.get_thrust(self.voltage, self.at20)

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()

        if isinstance(self.voltage, sympy.Symbol):
            bounds[self.voltage.name] = (self.min_voltage, self.max_voltage)
        else:
            assert self.min_voltage <= self.voltage <= self.max_voltage

        return bounds

    def report(self) -> Dict[str, Any]:
        report = {
            self.prefix + "_voltage": self.voltage,
            self.prefix + "_current": self.current,
            self.prefix + "_power": self.power,
            self.prefix + "_thrust": self.thrust,
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report


def napkin2():
    if False:
        battery = SimpleBatteryModel(
            series_count=1,
            # battery_name="TurnigyGraphene6000mAh6S75C",
            # parallel_count=2,
            battery_name="Tattu30C12000mAh6S1P",
            parallel_count=1,
        )
        # print(battery.bounds())
        # print(battery.report())
    else:
        battery = ParetoBatteryModel(
            prefix="battery",
        )

    motor_prop_count = 4
    motor_prop = MotorPropModel(
        {
            "motor_name": "t_motor_MN3510KV700",
            "propeller_name": "apc_propellers_9_625x3_75N",
            "weight": 0.148889479,
            "min_voltage": 0.035,
            "min_omega_rpm": 7.0,
            "min_thrust": 0.0,
            "min_power": 0.02,
            "min_current": 0.53,
            "min_omega_rpm_at20": 7.39,
            "min_thrust_at20": -0.01,
            "min_power_at20": 0.02,
            "min_current_at20": 0.51,
            "med_voltage": 12.747499999999999,
            "med_omega_rpm": 8701.88,
            "med_thrust": 5.13,
            "med_power": 84.87,
            "med_current": 6.66,
            "med_omega_rpm_at20": 8801.86,
            "med_thrust_at20": 1.01,
            "med_power_at20": 46.54,
            "med_current_at20": 3.65,
            "max_voltage": 25.459999999999997,
            "max_omega_rpm": 17061.14,
            "max_thrust": 20.07,
            "max_power": 582.6,
            "max_current": 22.88,
            "max_omega_rpm_at20": 16932.88,
            "max_thrust_at20": 16.05,
            "max_power_at20": 680.82,
            "max_current_at20": 26.74
        }
    )

    thrust_takeoff = ThrustModel(
        motor_prop=motor_prop,
        # voltage=battery.max_voltage,
        at20=False,
        prefix="thrust_takeoff",
    )
    # print(takeoff.bounds())
    # print(takeoff.report())

    thrust_flight = ThrustModel(
        motor_prop=motor_prop,
        # voltage=battery.max_voltage,
        at20=True,
        prefix="thrust_flight",
    )
    # print(flight.bounds())
    # print(flight.report())

    wing_count = 2
    wing = WingModel(
        naca_profile="NACA 0012",
        # max_load=25,  # N
        # chord=0.1,    # m
        # span=0.6,     # m
        prefix="wing",
    )
    # print(wing.bounds())
    # print(wing.report())

    wing_flight = LiftModel(
        wing=wing,
        # speed=25.26121,
        # angle=10.0,
        prefix="wing_flight",
    )
    # print(liftdrag.bounds())
    # print(liftdrag.report())

    cargo_weight = 0.5
    aircraft_weight = 1.47 + cargo_weight + battery.weight \
        + motor_prop_count * motor_prop.weight \
        + wing_count * wing.weight                         # kg

    aircraft_frontal_area = 0.057456                       # m^2
    aircraft_frontal_drag = 0.5 * AIR_DENSITY * \
        aircraft_frontal_area * wing_flight.speed ** 2     # N

    takeoff_duration = 2 * 100.0                           # s
    takeoff_capacity = thrust_takeoff.current * \
        motor_prop_count * takeoff_duration / 3600.0       # Ah

    flight_distance = 5100.0                               # m
    flight_duration = flight_distance / wing_flight.speed  # s
    flight_capacity = thrust_flight.current * \
        motor_prop_count * flight_duration / 3600.0        # Ah

    capacity_frac = (takeoff_capacity + flight_capacity) / battery.capacity

    bounds = {
        **battery.bounds(),
        **thrust_takeoff.bounds(),
        **thrust_flight.bounds(),
        **wing.bounds(),
        **wing_flight.bounds(),
    }

    constraints = {
        **battery.constraints(),
        **wing_flight.constraints(),
        "equ_takeoff_current": battery.current >= thrust_takeoff.current * motor_prop_count,
        "equ_takeoff_voltage": battery.voltage >= thrust_takeoff.voltage,
        "equ_takeoff_thrust": thrust_takeoff.thrust * motor_prop_count >= aircraft_weight * GRAVITATION,
        "equ_flight_current": battery.current >= thrust_flight.current * motor_prop_count,
        "equ_flight_voltage": battery.voltage >= thrust_flight.voltage,
        "equ_flight_lift": wing_flight.lift * wing_count >= aircraft_weight * GRAVITATION,
        "equ_flight_drag": wing_flight.drag * wing_count + aircraft_frontal_drag <= thrust_flight.thrust * motor_prop_count,
        "equ_total_capacity": capacity_frac < 0.5,
    }

    reports = {
        **battery.report(),
        **thrust_takeoff.report(),
        **thrust_flight.report(),
        **wing.report(),
        **wing_flight.report(),
        "motor_prop_count": motor_prop_count,
        "wing_count": wing_count,
        "aircraft_weight": aircraft_weight,
        "takeoff_duration": takeoff_duration,
        "takeoff_capacity": takeoff_capacity,
        "flight_duration": flight_duration,
        "flight_capacity": flight_capacity,
        "capacity_frac": capacity_frac,
    }

    print("bounds:\n", bounds)
    print("constraints:\n", constraints)
    print("reports:\n", reports)

    # generate random points
    num = 5000
    points = PointCloud.generate(bounds, num)
    constraints_func = PointFunc(constraints)
    reports_func = PointFunc(reports)

    for step in range(20):
        tol = [10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01]
        tol = tol[min(step, len(tol) - 1)]
        points.add_mutations(tol, num)

        points = points.newton_raphson(constraints_func, bounds, num_iter=10)
        points = points.prune_by_tolerances(
            constraints_func(points), tolerances=tol)
        points = points.prune_bounding_box(bounds)
        if True:
            points = points.prune_close_points2({
                "wing_flight_speed": 0.1,
                "aircraft_weight": 0.01,
                "wing_chord": 0.01,
                "wing_span": 0.01,
                "wing_max_load": 1,
            })

        points = points.extend(reports_func(points, equs_as_float=False))
        points = points.extend(constraints_func(points, equs_as_float=True))

        if True:
            points = points.prune_pareto_front2({
                "wing_flight_speed": 1.0,
                "aircraft_weight": -1.0,
            })

        print("step {} designs: {}".format(step, points.num_points))
        if points.num_points:
            print(json.dumps(points.row(0), indent=2, sort_keys=True))

    points.save("napkin2.csv")
    points.plot2d("aircraft_weight", "wing_flight_speed")


def napkin1():
    motor_prop_count = 8
    if False:
        motor_prop = MotorPropModel(
            motor_name="MAGiDRIVE150",
            propeller_name="62x5_2_3200_46_1150",
            weight=35.831,
            voltage0=400.0,
            current0=42.09,
            thrust0=510.54,
            voltage1=600.0,
            current1=81.34,
            thrust1=1088.06,
            voltage2=828.8,
            current2=144.13,
            thrust2=1944.44,
            min_voltage=348.0,
            max_voltage=845.1,
        )
    elif True:
        motor_prop = MotorPropModel(
            motor_name="MAGiDRIVE300",
            propeller_name="90x8_2_2000_41_2000",
            weight=62.228,
            voltage0=300.0,
            current0=85.11,
            thrust0=841.32,
            voltage1=400.0,
            current1=131.89,
            thrust1=1415.16,
            voltage2=500.0,
            current2=188.34,
            thrust2=2089.92,
            min_voltage=268.0,
            max_voltage=724.45,
        )
    elif False:
        motor_prop = MotorPropModel(
            motor_name="HPDM250",
            propeller_name="62x5_2_3200_46_1150",
            weight=15.831,
            voltage0=155.4,
            current0=314.58,
            thrust0=1137.29,
            voltage1=207.2,
            current1=475.37,
            thrust1=1396.82,
            voltage2=259.0,
            current2=657.33,
            thrust2=1900.74,
        )
    elif True:
        motor_prop = MotorPropModel(
            motor_name="MAGiDRIVE150",
            propeller_name="76x6_2_2400_41_1420",
            weight=36.418,
            voltage0=362.6,
            current0=65.64,
            thrust0=731.42,
            voltage1=518.0,
            current1=114.44,
            thrust1=1337.72,
            voltage2=673.4,
            current2=173.2,
            thrust2=2043.79,
            min_voltage=288.0,
            max_voltage=694.28,
        )
    else:
        motor_prop = MotorPropModel(
            motor_name="MAGiDRIVE150",
            propeller_name="90x8_2_2000_41_2000",
            weight=37.228,
            voltage0=362.6,
            current0=104.22,
            thrust0=1009.43,
            voltage1=466.2,
            current1=148.99,
            thrust1=1118.71332,
            voltage2=569.8,
            current2=198.13,
            thrust2=1988.97,
        )

    battery_count = 2
    if True:
        battery = SimpleBatteryModel(
            battery_name="Tattu 25Ah Li",
            series_count=11,
            # parallel_count=5,
        )
    else:
        battery = SimpleBatteryModel(
            battery_name="Eagle 11 Ah Li",
            series_count=51,
            # parallel_count=4,
        )

    wing_count = 2
    wing = WingModel(
        naca_profile="0015",
        # max_load=10000,
        # chord=1.2,
        # span=7.0,
    )

    fuselage_mass = 400                         # kg
    fuselage_frontal_area = 2012345 * 1e-6      # m^2

    aircraft_mass = fuselage_mass + \
        battery.weight * battery_count + \
        motor_prop.weight * motor_prop_count + \
        wing.weight * wing_count
    aircraft_weight = aircraft_mass * GRAVITATION
    aircraft_capacity = battery.capacity * battery_count * 0.8

    flying_motor = ThrustModel(
        motor_prop=motor_prop,
        voltage=None,
        min_voltage=motor_prop.min_voltage,
        #        max_voltage=battery.max_voltage,
        max_voltage=min(motor_prop.max_voltage, battery.voltage),
        prefix="flying_motor"
    )

    flying_wing = LiftModel(wing=wing,
                            # speed=50.0,
                            # angle=0.765,
                            prefix="flying_wing")

    flying_time = sympy.Symbol("flying_time")
    flying_distance = flying_time * flying_wing.speed
    flying_fuselage_drag = 0.5 * AIR_DENSITY * fuselage_frontal_area * \
        flying_wing.speed ** 2

    flying_total_lift = flying_wing.lift * wing_count
    flying_total_thrust = flying_motor.thrust * motor_prop_count
    flying_total_drag = flying_fuselage_drag + flying_wing.drag * wing_count

    hover_motor = ThrustModel(
        motor_prop=motor_prop,
        voltage=None,
        min_voltage=motor_prop.min_voltage,
        #        max_voltage=battery.max_voltage,
        max_voltage=min(motor_prop.max_voltage, battery.voltage),
        prefix="hover_motor"
    )

    hover_time = sympy.Symbol("hover_time")
    hover_total_thrust = hover_motor.thrust * motor_prop_count

    constraints = {
        **flying_wing.constraints(),
        "equ_battery_volume": battery.volume * battery_count <= wing.available_volume * wing_count,
        "equ_hover_current": battery.current * battery_count >= hover_motor.current * motor_prop_count,
        "equ_hover_voltage": battery.voltage >= hover_motor.voltage,
        "equ_hover_capacity": aircraft_capacity * 3600.0 >= hover_motor.current * motor_prop_count * hover_time,
        "equ_hover_thrust": hover_total_thrust >= aircraft_weight,
        "equ_flying_current": battery.current * battery_count >= flying_motor.current * motor_prop_count,
        "equ_flying_voltage": battery.voltage >= flying_motor.voltage,
        "equ_flying_capacity": aircraft_capacity * 3600.0 >= flying_motor.current * motor_prop_count * flying_time,
        "equ_flying_lift": flying_total_lift >= aircraft_weight,
        "equ_flying_drag": flying_total_thrust >= flying_total_drag,
    }

    bounds = {
        **flying_motor.bounds(),
        **hover_motor.bounds(),
        **battery.bounds(),
        **wing.bounds(),
        **flying_wing.bounds(),
        "flying_time": (0.0, 2*1000.0),
        "hover_time": (0.0, 2*400.0),
    }

    report_func = PointFunc({
        **flying_motor.report(),
        **hover_motor.report(),
        **battery.report(),
        **wing.report(),
        **flying_wing.report(),
        "motor_prop_count": motor_prop_count,
        "wing_count": wing_count,
        "battery_count": battery_count,
        "flying_distance": flying_distance,
        "aircraft_mass": aircraft_mass,
        "aircraft_weight": aircraft_weight,
        "volume_percent": 100.0 * battery.volume / wing.available_volume,
        "flying_fuselage_drag": flying_fuselage_drag,
        "flying_total_lift": flying_total_lift,
        "flying_total_drag": flying_total_drag,
        "flying_total_thrust": flying_total_thrust,
        "hover_total_thrust": hover_total_thrust,
    })

    # generate random points
    num = 5000
    points = PointCloud.generate(bounds, num)
    constraints_func = PointFunc(constraints)

    for step in range(100):
        tol = 100.0 if step == 0 else 10.0 if step == 1 else 1.0 if step == 2 else 0.1

        points.add_mutations(tol, num)

        points = points.newton_raphson(constraints_func, bounds, num_iter=10)

        points = points.prune_by_tolerances(
            constraints_func(points), tolerances=tol)

        points = points.extend(report_func(points, equs_as_float=False))
        points = points.extend(constraints_func(points, equs_as_float=True))

        if True:
            points = points.prune_pareto_front2({
                "flying_distance": 1.0,
                "hover_time": 1.0,
                # "aircraft_weight": -1.0,
            })

        if False:
            points = points.prune_close_points2(resolutions=0.1)

        print("step {} designs: {}".format(step, points.num_points))
        if points.num_points:
            print(json.dumps(points.row(0), indent=2, sort_keys=True))

        points.save("pareto_step_{}.csv".format(step))

    points.save("vudoo_designs.csv")
    points.plot2d("flying_distance", "hover_time")


def plot_pareto_steps():
    import matplotlib.pyplot as plt
    var1 = "flying_distance"
    var2 = "hover_time"

    fig, ax1 = plt.subplots()
    for step in [0, 1, 2, 4, 9, 99]:
        points = PointCloud.load("pareto_step_{}.csv".format(step))
        points = points.prune_bounding_box({'hover_time': (400, 1000)})

        c = max(170 - step, 0)
        ax1.scatter(
            x=points[var1].numpy(),
            y=points[var2].numpy(),
            s=25,
            # color="#{:02x}{:02x}{:02x}".format(c, c, c),
            label="step {}".format(step+1))

    ax1.set_xlabel("Flying distance (m)")
    ax1.set_ylabel("Hover time (s)")
    ax1.set_title("Flying distance vs hover time Pareto-front")

    ax1.grid()
    ax1.legend()

    plt.show()


def test():
    battery = SimpleBatteryModel(
        battery_name="TurnigyGraphene6000mAh6S75C",
        series_count=1,
        parallel_count=2,
    )
    # print(battery.bounds())
    print(battery.report())

    # front_left_wing
    wing = WingModel(
        naca_profile="NACA 0012",
        max_load=30,     # N
        chord=0.150,     # m
        span=0.450,      # m
    )
    # print(wing.bounds())
    print(wing.report())


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('design', choices=[
        "napkin1",
        "napkin2",
        "plot-pareto-steps",
        "test",
    ])
    args = parser.parse_args(args)

    if args.design == "napkin1":
        napkin1()
    elif args.design == "napkin2":
        napkin2()
    elif args.design == "plot-pareto-steps":
        plot_pareto_steps()
    elif args.design == "test":
        test()
    else:
        raise ValueError("unknown design")


if __name__ == '__main__':
    run()
