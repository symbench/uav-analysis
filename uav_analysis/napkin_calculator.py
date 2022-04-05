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
from numpy import isin
import sympy

from .components import BATTERIES, WINGS

from constraint_prog.point_cloud import PointCloud, PointFunc
from constraint_prog.sympy_func import SympyFunc


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

    air_density = 1.225                # kg/m^3
    frontal_area = 2012345 * 1e-6      # m^2
    fuselage = {
        "weight": 400,
        "drag_force": 0.5 * air_density * frontal_area * flying_speed ** 2,
    }

    aircraft_weight = fuselage["weight"] + battery_pack["weight"] + \
        forward_motor_prop["weight"] + \
        lifting_motor_prop["weight"] + moving_wing["weight"]
    hower_time = battery_pack["energy"] / lifting_motor_prop["power"] * 3600.0
    flying_time = battery_pack["energy"] / forward_motor_prop["power"] * 3600.0
    flying_distance = flying_time * flying_speed

    gravitation = 9.81                 # m/s^2
    constraints = {
        "available_volume_equ": battery_pack["volume"] <= moving_wing["available_volume"],
        "hower_current_equ": battery_pack["current"] >= lifting_motor_prop["current"],
        "hower_thrust_equ": lifting_motor_prop["thrust"] >= aircraft_weight * gravitation,
        "flying_current_equ": battery_pack["current"] >= forward_motor_prop["current"],
        "flying_lift_equ": moving_wing["lift_force"] >= aircraft_weight * gravitation,
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
        "hower_time": hower_time,
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
            "hower_current_equ": 0.1,
            "hower_thrust_equ": 1.0,
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

    air_density = 1.225                # kg/m^3
    frontal_area = 2012345 * 1e-6      # m^2
    fuselage = {
        "weight": 350,
        "drag_force": 0.5 * air_density * frontal_area * flying_speed ** 2,
    }

    aircraft_weight = fuselage["weight"] + battery_pack["weight"] + \
        motor_pack["weight"] + wing_pack["weight"]
    flying_time = battery_pack["energy"] / motor_pack["power"] * 3600.0
    flying_distance = flying_time * flying_speed

    gravitation = 9.81                 # m/s^2
    constraints = {
        "available_volume_equ": battery_pack["volume"] <= wing_pack["available_volume"],
        "motor_current_equ": battery_pack["current"] >= motor_pack["current"],
        "hower_thrust_equ": motor_pack["thrust"] >= aircraft_weight * gravitation,
        "flying_lift_equ": wing_pack["lift_force"] >= aircraft_weight * gravitation,
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


class BatteryPack():
    def __init__(self,
                 battery_name: str,
                 pack_count: int,
                 series_count: Optional[int] = None,
                 parallel_count: Optional[int] = None):

        battery = BATTERIES[battery_name]
        self.battery_name = battery_name
        self.single_voltage = float(battery["Min Voltage [V]"])
        self.single_capacity = float(battery["Capacity [Ah]"])
        self.discharge_rate = float(battery["Cont. Discharge Rate [C]"])
        self.single_current = self.single_capacity * self.discharge_rate
        self.single_energy = self.single_capacity * self.single_voltage
        self.single_weight = float(battery["Weight [kg]"])
        self.single_volume = float(battery["Volume [mm^3]"]) * 1e-9

        self.pack_count = pack_count

        if series_count is None:
            series_count = sympy.Symbol("battery_series_count")
        self.series_count = series_count

        if parallel_count is None:
            parallel_count = sympy.Symbol("battery_parallel_count")
        self.parallel_count = parallel_count

        self.total_voltage = self.single_voltage * self.series_count
        self.total_current = self.single_current * \
            self.parallel_count * self.pack_count
        self.total_capacity = self.single_capacity * \
            self.parallel_count * self.pack_count
        self.total_energy = self.single_energy * \
            self.series_count * self.parallel_count * self.pack_count
        self.total_weight = self.single_weight * \
            self.series_count * self.parallel_count * self.pack_count
        self.total_volume = self.single_volume * \
            self.series_count * self.parallel_count * self.pack_count

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()
        if isinstance(self.series_count, sympy.Symbol):
            bounds[self.series_count.name] = (1.0, 100.0)
        if isinstance(self.parallel_count, sympy.Symbol):
            bounds[self.parallel_count.name] = (1.0, 10.0)
        return bounds

    def report(self) -> Dict[str, Any]:
        report = {
            "battery_series_count": self.series_count,
            "battery_parallel_count": self.parallel_count,
            "battery_total_voltage": self.total_voltage,
            "battery_total_capacity": self.total_capacity,
            "battery_total_current": self.total_current,
            "battery_total_energy": self.total_energy,
            "battery_total_weight": self.total_weight,
            "battery_total_volume": self.total_volume,
            "battery_pack_count": self.pack_count,
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
            bounds[self.max_load.name] = (0.0, 1e9)
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

        rho = 1.225  # air density
        qbarprime_w = 0.5 * rho * self.speed ** 2

        self.lift = wing.surface_area * qbarprime_w * C_Lw
        self.drag = wing.surface_area * qbarprime_w * C_Dw

        self.max_load = wing.max_load

    def bounds(self) -> Dict[str, Tuple[float, float]]:
        bounds = dict()

        if isinstance(self.speed, sympy.Symbol):
            bounds[self.speed.name] = (0.0, 50.0)
        else:
            assert 0 <= self.speed <= 50.0

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
            self.prefix + "load_equ": self.lift ** 2 + self.drag ** 2 <= self.max_load ** 2
        }


def vudoo_napkin():
    battery_pack = BatteryPack(
        battery_name="Tattu 25Ah Li",
        pack_count=2,
        series_count=16,
        # parallel_count=6,
    )

    wing = WingModel(
        naca_profile="0015",
        max_load=20000,
        # chord=1.4,
        # span=8.0,
    )

    wing_count = 2

    flying = LiftModel(wing=wing,
                       # speed=50.0,
                       # angle=0.765,
                       prefix="flying")

    motor_prop = {
        "motor_name": "MAGiDRIVE150",
        "propeller_name": "62x5_2_3200_46_1150",
        "weight": 35.831,
        "propeller_diameter": 1.5748,
        "voltage": 828.8,
        "thrust": 1944.44,
        "power": 119456.52,
        "current": 144.13,
        "max_voltage": 845.1,
        # "max_thrust": 1582.81,
        "max_power": 126315.79,
    }

    motor_count = 16

    if motor_count is None:
        motor_count = sympy.Symbol("motor_count")
    motor_pack = {
        "weight": motor_prop["weight"] * motor_count,
        "thrust": motor_prop["thrust"] * motor_count,
        "power": motor_prop["power"] * motor_count,
        "current": motor_prop["current"] * motor_count,
    }

    flying_time = battery_pack.total_energy / motor_pack["power"] * 3600.0
    flying_distance = flying_time * flying.speed

    air_density = 1.225                # kg/m^3
    frontal_area = 2012345 * 1e-6      # m^2
    fuselage = {
        "weight": 350,
        "drag_force": 0.5 * air_density * frontal_area * flying.speed ** 2,
    }

    aircraft_weight = fuselage["weight"] + battery_pack.total_weight + \
        motor_pack["weight"] + wing.weight * wing_count

    gravitation = 9.81                 # m/s^2
    constraints = {
        **flying.equations(),
        "available_volume_equ": battery_pack.total_volume <= wing.available_volume * wing_count,
        "motor_current_equ": battery_pack.total_current >= motor_pack["current"],
        "hower_thrust_equ": motor_pack["thrust"] >= aircraft_weight * gravitation,
        "flying_lift_equ": flying.lift * wing_count >= aircraft_weight * gravitation,
        "flying_thrust_equ": motor_pack["thrust"] >= fuselage["drag_force"] + flying.drag * wing_count,
        "flying_time_equ": flying_time <= 1000.0,
    }

    bounds = {
        **battery_pack.bounds(),
        **wing.bounds(),
        **flying.bounds(),
    }

    if isinstance(motor_count, sympy.Symbol):
        bounds["motor_count"] = (1.0, 30.0)

    report_func = PointFunc({
        **battery_pack.report(),
        **wing.report(),
        **flying.report(),
        "flying_time": flying_time,
        "flying_distance": flying_distance,
        "aircraft_weight": aircraft_weight,
        "battery_pack_volume_percent": 100.0 * battery_pack.total_volume / (wing.available_volume * wing_count),
        "motor_pack_current": motor_pack["current"],
        "motor_pack_power": motor_pack["power"],
        "motor_pack_thrust": motor_pack["thrust"],
        "motor_pack_weight": motor_pack["weight"],
        "fuselage_drag_force": fuselage["drag_force"],
    })

    # generate random points
    num = 5000
    points = PointCloud.generate(bounds, num)
    constraints_func = PointFunc(constraints)

    for step in range(5):
        points.add_mutations(2.0, num)

        points = points.newton_raphson(constraints_func, bounds, num_iter=10)
        points = points.extend(constraints_func(points))

        points = points.prune_by_tolerances(
            constraints_func(points), tolerances=0.1)

        if False:
            points = points.prune_close_points2(resolutions=0.1)

        points = points.extend(report_func(points, equs_as_float=False))
        if True:
            points = points.prune_pareto_front2({
                "flying_distance": 1.0,
                # "flying_drag": -1.0,
                "aircraft_weight": -1.0,
            })

        print("designs: {}".format(points.num_points))
        if points.num_points:
            print(json.dumps(points.row(0), indent=2))

    points.plot2d("flying_distance", "aircraft_weight")


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('test', choices=[
        "shovel-big",
        "shovel-small",
        "tailsitter-small",
        "tailsitter-big",
        "vudoo",
    ])

    args = parser.parse_args(args)

    if args.test == "shovel-big":
        shovel_big()
    elif args.test == "shovel-small":
        shovel_small()
    elif args.test == "tailsitter-big":
        tailsitter_big()
    elif args.test == "tailsitter-small":
        tailsitter_small()
    elif args.test == "vudoo":
        vudoo_napkin()
    else:
        raise ValueError("unknown generator")


if __name__ == '__main__':
    run()
