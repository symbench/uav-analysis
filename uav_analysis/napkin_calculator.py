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

from typing import Any, Dict, Optional, Tuple

import json
import math
import numpy
import sympy

from .components import BATTERIES, WINGS

from constraint_prog.point_cloud import PointCloud, PointFunc
from constraint_prog.sympy_func import SympyFunc


AIR_DENSITY = 1.225                # kg/m^3
GRAVITATION = 9.81                 # m/s^2


def shovel_napkin(motor_prop: Dict[str, float],
                  batt: Dict[str, float],
                  wing: Dict[str, float],
                  forward_count: Optional[int] = None):
    assert forward_count is None or isinstance(forward_count, int)

    series_count = int(math.floor(motor_prop["max_voltage"] / batt["voltage"]))
    parallel_count = sympy.Symbol("parallel_count")
    battery_pack = {
        "weight": batt["weight"] * series_count * parallel_count,
        "volume": batt["volume"] * series_count * parallel_count,
        "energy": batt["energy"] * series_count * parallel_count,
        "voltage": batt["voltage"] * series_count,
        "current": batt["current"] * parallel_count,
    }

    lifting_count = sympy.Symbol("lifting_count")
    lifting_motor_prop = {
        "weight": motor_prop["weight"] * lifting_count,
        "thrust": motor_prop["thrust"] * lifting_count,
        "power": motor_prop["power"] * lifting_count,
        "current": motor_prop["current"] * lifting_count,
    }

    if not isinstance(forward_count, int):
        forward_count = sympy.Symbol("forward_count")
    forward_motor_prop = {
        "weight": motor_prop["weight"] * forward_count,
        "thrust": motor_prop["thrust"] * forward_count,
        "power": motor_prop["power"] * forward_count,
        "current": motor_prop["current"] * forward_count,
    }

    wing_count = 2
    flying_speed = sympy.Symbol("flying_speed")  # m/s
    moving_wing = {
        "weight": wing["weight"] * wing_count,
        "available_volume": wing["available_volume"] * wing_count,
        "lift_force": wing_count * wing["lift_50mps"] * (flying_speed / 50.0) ** 2,
        "drag_force": wing_count * wing["drag_50mps"] * (flying_speed / 50.0) ** 2,
    }

    frontal_area = 2012345 * 1e-6      # m^2
    fuselage = {
        "weight": 400,
        "drag_force": 0.5 * AIR_DENSITY * frontal_area * flying_speed ** 2,
    }

    aircraft_weight = fuselage["weight"] + battery_pack["weight"] + \
        forward_motor_prop["weight"] + \
        lifting_motor_prop["weight"] + moving_wing["weight"]
    hover_time = battery_pack["energy"] / lifting_motor_prop["power"] * 3600.0
    flying_time = battery_pack["energy"] / forward_motor_prop["power"] * 3600.0
    flying_distance = flying_time * flying_speed

    constraints = {
        "available_volume_equ": battery_pack["volume"] <= moving_wing["available_volume"],
        "hover_current_equ": battery_pack["current"] >= lifting_motor_prop["current"],
        "hover_thrust_equ": lifting_motor_prop["thrust"] >= aircraft_weight * GRAVITATION,
        "flying_current_equ": battery_pack["current"] >= forward_motor_prop["current"],
        "flying_lift_equ": moving_wing["lift_force"] >= aircraft_weight * GRAVITATION,
        "flying_thrust_equ": forward_motor_prop["thrust"] >= fuselage["drag_force"] + moving_wing["drag_force"],
    }

    bounds = {
        "parallel_count": (1.0, 1e6),
        "lifting_count": (1.0, 1e6),
        "forward_count": (1.0, 1e6),
        "flying_speed": (0.0, 50.0),
    }

    if isinstance(forward_count, int):
        del bounds["forward_count"]

    report_func = PointFunc({
        "series_count": series_count,
        "aircraft_weight": aircraft_weight,
        "hover_time": hover_time,
        "flying_time": flying_time,
        "flying_distance": flying_distance,
        "battery_pack_voltage": battery_pack["voltage"],
        "battery_pack_volume": battery_pack["volume"],
        "battery_pack_current": battery_pack["current"],
        "battery_pack_energy": battery_pack["energy"],
        "lifting_motor_current": lifting_motor_prop["current"],
        "lifting_motor_power": lifting_motor_prop["power"],
        "lifting_motor_thrust": lifting_motor_prop["thrust"],
        "forward_motor_current": forward_motor_prop["current"],
        "forward_motor_power": forward_motor_prop["power"],
        "forward_motor_thrust": forward_motor_prop["thrust"],
        "wing_available_volume": moving_wing["available_volume"],
    })

    # generate random points
    num = 5000
    points = PointCloud.generate(bounds, num)
    constraints_func = PointFunc(constraints)

    for step in range(5):
        points.add_mutations(2.0, num)

        points = points.newton_raphson(constraints_func, bounds, num_iter=10)
        points = points.prune_by_tolerances(constraints_func(points), {
            "available_volume_equ": 0.01,
            "hover_current_equ": 0.1,
            "hover_thrust_equ": 1.0,
            "flying_current_equ": 0.1,
            "flying_lift_equ": 1.0,
            "flying_thrust_equ": 1.0,
        })
        if False:
            points = points.prune_close_points2(resolutions={
                "parallel_count": 0.1,
                "lifting_count": 0.1,
                "forward_count": 0.1,
                "flying_speed": 0.1,
            })

        points = points.extend(report_func(points))
        if True:
            points = points.prune_pareto_front2({
                "flying_distance": 1.0,
                # "flying_speed": 1.0,
            })

        print("designs: {}".format(points.num_points))
        if points.num_points:
            print(json.dumps(points.row(0), indent=2))

    # points.plot2d("flying_distance", "flying_speed")


def shovel_small():
    motor_prop = {
        "motor_name": "KDE13218XF-105",
        "propeller_name": "34x3_2_4600_41_250",
        "weight": 2.313,
        "propeller_diameter": 0.8636,
        "voltage": 25.0,
        "thrust": 106.04,
        "power": 1743.54,
        "current": 69.74,
        "max_voltage": 38.29,
        "max_thrust": 40.39,
        "max_power": 6045.95,
        "max_current": 157.89,
    }

    batt = {
        "weight": 0.004,
        "volume": 8.75e-05,
        "energy": 11.1,
        "voltage": 11.1,
        "current": 25.0,
        "name": "Vitaly Beta"
    }

    wing = {
        "weight": 29.487522,
        "lift_50mps": 9287.95,
        "drag_50mps": 440.14,
        "available_volume": 0.360,
        "profile_name": "NACA 2418",
        "chord": 1.0,
        "span": 10.0
    }

    shovel_napkin(motor_prop, batt, wing)


def shovel_big():
    motor_prop = {
        "motor_name": "MAGiDRIVE150",
        "propeller_name": "62x5_2_3200_46_1150",
        "weight": 35.831,
        "propeller_diameter": 1.5748,
        "voltage": 700.0,
        "thrust": 1444.34,
        "power": 74098.07,
        "current": 105.85,
        "max_voltage": 845.1,
        "max_thrust": 1422.26,
        "max_power": 126315.79,
        "max_current": 149.47,
    }

    batt = {
        "weight": 0.004,
        "volume": 8.75e-05,
        "energy": 11.1,
        "voltage": 11.1,
        "current": 25.0,
        "name": "Vitaly Beta"
    }

    wing = {
        "weight": 29.487522,
        "lift_50mps": 9287.95,
        "drag_50mps": 440.14,
        "available_volume": 0.360,
        "profile_name": "NACA 2418",
        "chord": 1.0,
        "span": 10.0
    }

    shovel_napkin(motor_prop, batt, wing, forward_count=None)


def tailsitter_napkin(motor_prop: Dict[str, float],
                      batt: Dict[str, float],
                      wing: Dict[str, float],
                      motor_count: Optional[int] = None):

    series_count = int(math.floor(motor_prop["max_voltage"] / batt["voltage"]))

    parallel_count = sympy.Symbol("parallel_count")
    battery_pack = {
        "weight": batt["weight"] * series_count * parallel_count,
        "volume": batt["volume"] * series_count * parallel_count,
        "energy": batt["energy"] * series_count * parallel_count,
        "voltage": batt["voltage"] * series_count,
        "current": batt["current"] * parallel_count,
    }

    if motor_count is None:
        motor_count = sympy.Symbol("motor_count")
    motor_pack = {
        "weight": motor_prop["weight"] * motor_count,
        "thrust": motor_prop["thrust"] * motor_count,
        "power": motor_prop["power"] * motor_count,
        "current": motor_prop["current"] * motor_count,
    }

    total_span = sympy.Symbol("total_span")  # m
    flying_speed = sympy.Symbol("flying_speed")  # m/s
    wing_pack = {
        "weight": wing["weight"] / wing["span"] * total_span,
        "available_volume": wing["available_volume"] / wing["span"] * total_span,
        "lift_force": wing["lift_50mps"] * (flying_speed / 50.0) ** 2 / wing["span"] * total_span,
        "drag_force": wing["drag_50mps"] * (flying_speed / 50.0) ** 2 / wing["span"] * total_span,
    }

    frontal_area = 2012345 * 1e-6      # m^2
    fuselage = {
        "weight": 350,
        "drag_force": 0.5 * AIR_DENSITY * frontal_area * flying_speed ** 2,
    }

    aircraft_weight = fuselage["weight"] + battery_pack["weight"] + \
        motor_pack["weight"] + wing_pack["weight"]
    flying_time = battery_pack["energy"] / motor_pack["power"] * 3600.0
    flying_distance = flying_time * flying_speed

    constraints = {
        "available_volume_equ": battery_pack["volume"] <= wing_pack["available_volume"],
        "motor_current_equ": battery_pack["current"] >= motor_pack["current"],
        "hover_thrust_equ": motor_pack["thrust"] >= aircraft_weight * GRAVITATION,
        "flying_lift_equ": wing_pack["lift_force"] >= aircraft_weight * GRAVITATION,
        "flying_thrust_equ": motor_pack["thrust"] >= fuselage["drag_force"] + wing_pack["drag_force"],
        "flying_time_equ": flying_time <= 1000.0,
    }

    bounds = {
        "parallel_count": (1.0, 1e6),
        "total_span": (10.0, 50.0),
        "flying_speed": (0.0, 50.0),
    }

    if isinstance(motor_count, sympy.Symbol):
        bounds["motor_count"] = (1.0, 30.0)

    report_func = PointFunc({
        "series_count": series_count,
        "aircraft_weight": aircraft_weight,
        "flying_time": flying_time,
        "flying_distance": flying_distance,
        "battery_pack_voltage": battery_pack["voltage"],
        "battery_pack_current": battery_pack["current"],
        "battery_pack_percent": 100.0 * battery_pack["volume"] / wing_pack["available_volume"],
        "battery_pack_volume": battery_pack["volume"],
        "battery_pack_current": battery_pack["current"],
        "battery_pack_energy": battery_pack["energy"],
        "battery_pack_weight": battery_pack["weight"],
        "motor_pack_current": motor_pack["current"],
        "motor_pack_power": motor_pack["power"],
        "motor_pack_thrust": motor_pack["thrust"],
        "motor_pack_weight": motor_pack["weight"],
        "wing_pack_lift_force": wing_pack["lift_force"],
        "wing_pack_drag_force": wing_pack["drag_force"],
        "wing_pack_weight": wing_pack["weight"],
        "wing_pack_available_volume": wing_pack["available_volume"],
        "fuselage_drag_force": fuselage["drag_force"],
    })

    # generate random points
    num = 5000
    points = PointCloud.generate(bounds, num)
    constraints_func = PointFunc(constraints)

    for step in range(5):
        points.add_mutations(2.0, num)

        points = points.newton_raphson(constraints_func, bounds, num_iter=10)
        points = points.prune_by_tolerances(constraints_func(points), 0.1)

        if False:
            points = points.prune_close_points2(resolutions=0.1)

        points = points.extend(report_func(points))
        if True:
            points = points.prune_pareto_front2({
                "flying_distance": 1.0,
                # "motor_count": -1.0,
            })

        print("designs: {}".format(points.num_points))
        if points.num_points:
            print(json.dumps(points.row(0), indent=2))

    # points.plot2d("flying_distance", "motor_count")


def tailsitter_big():
    motor_prop = {
        "motor_name": "MAGiDRIVE150",
        "propeller_name": "62x5_2_3200_46_1150",
        "weight": 35.831,
        "propeller_diameter": 1.5748,
        "voltage": 700.0,
        "thrust": 1444.34,
        "power": 74098.07,
        "current": 105.85,
        "max_voltage": 845.1,
        "max_thrust": 1422.26,
        "max_power": 126315.79,
        "max_current": 149.47,
    }

    batt = {
        "weight": 0.004,
        "volume": 8.75e-05,
        "energy": 11.1,
        "voltage": 11.1,
        "current": 25.0,
        "name": "Vitaly Beta"
    }

    wing = {
        "weight": 29.487522,
        "lift_50mps": 9287.95,
        "drag_50mps": 440.14,
        "available_volume": 0.360,
        "profile_name": "NACA 2418",
        "chord": 1.0,
        "span": 10.0
    }

    tailsitter_napkin(motor_prop, batt, wing)


def tailsitter_small():
    motor_prop = {
        "motor_name": "KDE13218XF-105",
        "propeller_name": "34x3_2_4600_41_250",
        "weight": 2.313,
        "propeller_diameter": 0.8636,
        "voltage": 25.0,
        "thrust": 106.04,
        "power": 1743.54,
        "current": 69.74,
        "max_voltage": 38.29,
        "max_thrust": 40.39,
        "max_power": 6045.95,
        "max_current": 157.89,
    }

    batt = {
        "weight": 0.004,
        "volume": 8.75e-05,
        "energy": 11.1,
        "voltage": 11.1,
        "current": 25.0,
        "name": "Vitaly Beta"
    }

    wing = {
        "weight": 29.487522,
        "lift_50mps": 9287.95,
        "drag_50mps": 440.14,
        "available_volume": 0.360,
        "profile_name": "NACA 2418",
        "chord": 1.0,
        "span": 10.0
    }

    tailsitter_napkin(motor_prop, batt, wing)


class BatteryModel():
    def __init__(self,
                 battery_name: str,
                 series_count: Optional[int] = None,
                 parallel_count: Optional[int] = None,
                 prefix: str = "battery"):

        battery = BATTERIES[battery_name]
        self.battery_name = battery_name
        self.prefix = prefix

        single_max_voltage = float(battery["Min Voltage [V]"])
        single_capacity = float(battery["Capacity [Ah]"])
        single_weight = float(battery["Weight [kg]"])
        single_volume = float(battery["Volume [mm^3]"]) * 1e-9
        discharge_rate = float(battery["Cont. Discharge Rate [C]"])

        if series_count is None:
            series_count = sympy.Symbol(self.prefix + "_series_count")
        self.series_count = series_count

        if parallel_count is None:
            parallel_count = sympy.Symbol(self.prefix + "_parallel_count")
        self.parallel_count = parallel_count

        self.max_voltage = single_max_voltage * self.series_count
        self.capacity = single_capacity * self.parallel_count
        self.weight = single_weight * self.series_count * self.parallel_count
        self.volume = single_volume * self.series_count * self.parallel_count
        self.max_current = self.capacity * discharge_rate
        self.energy = self.max_voltage * self.capacity

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
            self.prefix + "_max_voltage": self.max_voltage,
            self.prefix + "_max_current": self.max_current,
            self.prefix + "_capacity": self.capacity,
            self.prefix + "_energy": self.energy,
            self.prefix + "_weight": self.weight,
            self.prefix + "_volume": self.volume,
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report


class WingModel():
    def __init__(self,
                 naca_profile: str,
                 max_load: Optional[float] = None,
                 chord: Optional[float] = None,
                 span: Optional[float] = None):
        assert len(naca_profile) == 4
        self.naca_profile = naca_profile
        self.thickness = float(naca_profile[2:4])

        if max_load is None:
            max_load = sympy.Symbol("wing_max_load")
        self.max_load = max_load

        if chord is None:
            chord = sympy.Symbol("wing_chord")
        self.chord = chord

        if span is None:
            span = sympy.Symbol("wing_span")
        self.span = span

        self.available_volume = WingModel.symbolic_available_volume(
            self.thickness, self.chord, self.span)

        self.weight = WingModel.symbolic_wing_weight(
            self.thickness, self.chord, self.span, self.max_load)

        self.wing_data = WingModel.get_wing_data(
            naca_profile, self.chord, self.span)

        self.surface_area = self.wing_data["surface_area"]
        self.min_angle = (self.wing_data["C_Lmin"] - self.wing_data["C_L0"]) / \
            self.wing_data["a"]
        self.max_angle = (self.wing_data["C_Lmax"] - self.wing_data["C_L0"]) / \
            self.wing_data["a"]

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()
        if isinstance(self.max_load, sympy.Symbol):
            bounds[self.max_load.name] = (0.0, 20000.0)
        if isinstance(self.chord, sympy.Symbol):
            bounds[self.chord.name] = (0.1, 5.0)
        else:
            assert 0.1 <= self.chord <= 5.0
        if isinstance(self.span, sympy.Symbol):
            bounds[self.span.name] = (0.1, 15.0)
        else:
            assert 0.1 <= self.span <= 15.0
        return bounds

    def report(self):
        report = {
            "wing_thickness": self.thickness,
            "wing_max_load": self.max_load,
            "wing_span": self.span,
            "wing_chord": self.chord,
            "wing_available_volume": self.available_volume,
            "wing_weight": self.weight,
            "wing_surface_area": self.surface_area,
            "wing_min_angle": self.min_angle,
            "wing_max_angle": self.max_angle,
        }
        for var in self.bounds():
            assert var in report
            del report[var]
        return report

    @staticmethod
    def symbolic_available_volume(thickness: Any, chord: Any, span: Any) -> Any:
        root_chord = chord
        tip_chord = chord

        A = root_chord/2
        B = thickness/100*A
        C = tip_chord/2
        D = thickness/100*C

        return 1/6*span*(A*B+C*D+((A+C)*(B+D)))

    @staticmethod
    def symbolic_wing_weight(thickness: Any, chord: Any, span: Any, max_load: Any) -> Any:
        chord = chord * 1000.0
        span = span * 1000.0

        chord1 = chord
        chord2 = chord
        taper_offset = 0  # assuming no taper offset

        N = 1.5  # safety factor
        VE = 50  # max airspeed
        # plan form area with taper accounted for
        s = (chord1 + chord2) / 2 * span
        ar = span ** 2 / s  # aspect ratio assuming non-constant chord wing
        sa = math.atan(abs(chord1 - chord2) *
                       taper_offset / span)  # weep angle
        mc = (chord1 + chord2) / 2  # mean chord
        tr = chord2 / chord1  # taper ratio
        weight = (
            0.4536
            * 96.948
            * (
                (max_load * 0.225 * N / (10 ** 5)) ** 0.65
                * (ar / math.cos(sa)) ** 0.57
                * (s / (100 * 92903)) ** 0.61
                * ((1 + (1 / tr)) / (2 * thickness / 100)) ** 0.36
                * (1 + (VE * 1.944 / 500)) ** 0.5
            )
            ** 0.993
        )
        return weight

    @staticmethod
    def get_wing_data(profile: str, chord: Any, span: Any):
        wing_dict = WINGS["NACA " + profile]
        chord1 = chord
        chord2 = chord

        MC = (chord1 + chord2) / 2  # Mean chord
        SA = MC * span  # Surface area = planform area
        # TR = min([chord1, chord2]) / max([chord1, chord2])  # Taper ratio
        TR = 1
        AR = span ** 2 / SA  # aspect ratio, modified defintion for tapered wings
        Hfun = 0.0524 * TR ** 4 - 0.15 * TR ** 3 + \
            0.1659 * TR ** 2 - 0.0706 * TR + 0.0119
        k = (1 + Hfun * AR) / (math.pi * AR)

        dcl_daoa_slope = float(wing_dict["dCl/dAoA Slope [1/deg.]"])
        aoa_l0 = float(wing_dict["AoA @ L0 [deg.]"])
        cl_max = float(wing_dict["CL Max"])
        cd_min = float(wing_dict["CD Min"])

        return {
            "a": dcl_daoa_slope,
            "C_L0": -dcl_daoa_slope * aoa_l0,
            "C_Lmax": cl_max,
            "C_Lmin": -cl_max,
            "C_D0": cd_min,
            "k": k,
            "C_Dfp": 1,
            "bias1": 1.0,
            "bias2": 0.5,
            "surface_area": SA,
        }


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
            bounds[self.speed.name] = (0.0, 2 * 50.0)
        else:
            assert 0 <= self.speed <= 2 * 50.0

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

    def equations(self):
        return {
            "equ_" + self.prefix + "_load": self.lift ** 2 + self.drag ** 2 <= 0.8 * self.max_load ** 2
        }


class MotorPropModel():
    def __init__(self,
                 motor_name: str,
                 propeller_name: str,
                 weight: float,
                 voltage0: float,
                 current0: float,
                 thrust0: float,
                 voltage1: float,
                 current1: float,
                 thrust1: float,
                 voltage2: float,
                 current2: float,
                 thrust2: float,
                 min_voltage: Optional[float] = None,
                 max_voltage: Optional[float] = None):
        assert voltage0 < voltage1 < voltage2
        assert current0 < current1 < current2
        assert thrust0 < thrust1 < thrust2

        self.motor_name = motor_name
        self.propeller_name = propeller_name
        self.weight = weight

        self.voltage0 = voltage0
        self.current0 = current0
        self.thrust0 = thrust0

        self.voltage1 = voltage1
        self.current1 = current1
        self.thrust1 = thrust1

        self.voltage2 = voltage2
        self.current2 = current2
        self.thrust2 = thrust2

        if min_voltage is None:
            self.min_voltage = min(voltage0, voltage1, voltage2)
        else:
            self.min_voltage = min_voltage

        if max_voltage is None:
            self.max_voltage = max(voltage0, voltage1, voltage2)
        else:
            self.max_voltage = max_voltage

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
        return c[0] + c[1] * x + c[2] * x ** 2

    def get_current(self, voltage: Any) -> Any:
        return MotorPropModel.quadratic_fit(
            self.voltage0, self.voltage1, self.voltage2,
            self.current0, self.current1, self.current2,
            voltage)

    def get_thrust(self, voltage: Any) -> Any:
        return MotorPropModel.quadratic_fit(
            self.voltage0, self.voltage1, self.voltage2,
            self.thrust0, self.thrust1, self.thrust2,
            voltage)


class ThrustModel():
    def __init__(self,
                 motor_prop: MotorPropModel,
                 voltage: Optional[float] = None,
                 min_voltage: Optional[float] = None,
                 max_voltage: Optional[float] = None,
                 prefix: str = "thrust"):

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

        self.current = motor_prop.get_current(self.voltage)
        self.power = self.voltage * self.current
        self.thrust = motor_prop.get_thrust(self.voltage)

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


def vudoo_napkin():
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
        battery = BatteryModel(
            battery_name="Tattu 25Ah Li",
            series_count=11,
            # parallel_count=5,
        )
    else:
        battery = BatteryModel(
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
        max_voltage=min(motor_prop.max_voltage, battery.max_voltage),
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
        max_voltage=min(motor_prop.max_voltage, battery.max_voltage),
        prefix="hover_motor"
    )

    hover_time = sympy.Symbol("hover_time")
    hover_total_thrust = hover_motor.thrust * motor_prop_count

    constraints = {
        **flying_wing.equations(),
        "equ_battery_volume": battery.volume * battery_count <= wing.available_volume * wing_count,
        "equ_hover_current": battery.max_current * battery_count >= hover_motor.current * motor_prop_count,
        "equ_hover_voltage": battery.max_voltage >= hover_motor.voltage,
        "equ_hover_capacity": aircraft_capacity * 3600.0 >= hover_motor.current * motor_prop_count * hover_time,
        "equ_hover_thrust": hover_total_thrust >= aircraft_weight,
        "equ_flying_current": battery.max_current * battery_count >= flying_motor.current * motor_prop_count,
        "equ_flying_voltage": battery.max_voltage >= flying_motor.voltage,
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
            #color="#{:02x}{:02x}{:02x}".format(c, c, c),
            label="step {}".format(step+1))

    ax1.set_xlabel("Flying distance (m)")
    ax1.set_ylabel("Hover time (s)")
    ax1.set_title("Flying distance vs hover time Pareto-front")

    ax1.grid()
    ax1.legend()

    plt.show()


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('design', choices=[
        "shovel-big",
        "shovel-small",
        "tailsitter-small",
        "tailsitter-big",
        "vudoo",
        "plot-pareto-steps",
    ])
    args = parser.parse_args(args)

    if args.design == "shovel-big":
        shovel_big()
    elif args.design == "shovel-small":
        shovel_small()
    elif args.design == "tailsitter-big":
        tailsitter_big()
    elif args.design == "tailsitter-small":
        tailsitter_small()
    elif args.design == "vudoo":
        vudoo_napkin()
    elif args.design == "plot-pareto-steps":
        plot_pareto_steps()
    else:
        raise ValueError("unknown design")


if __name__ == '__main__':
    run()
