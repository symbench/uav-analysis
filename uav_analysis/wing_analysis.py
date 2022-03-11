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

from typing import Any, List, Dict, Generator, Iterable

import sys
import csv
import math
import os
import re

from .bemp_combinations_hackathon2_uam import WINGS


NACA_PROFILE = re.compile(r"^NACA (\d)(\d)(\d\d)$")


def calc_wing_data(profile: str, chord1: float, chord2: float,
                   span: float) -> Dict[str, float]:
    """ Copied from run_fd_calc.py """
    wing_dict = WINGS[profile]

    MC = (chord1 + chord2) / 2  # Mean chord
    SA = MC * span  # Surface area = planform area
    TR = min([chord1, chord2]) / max([chord1, chord2])  # Taper ratio
    AR = span ** 2 / SA  # aspect ratio, modified defintion for tapered wings
    Hfun = 0.0524 * TR ** 4 - 0.15 * TR ** 3 + \
        0.1659 * TR ** 2 - 0.0706 * TR + 0.0119
    k = (1 + Hfun * AR) / (math.pi * AR)

    surface_area = (chord1 + chord2) / 2 * span
    dcl_daoa_slope = float(wing_dict["dCl/dAoA Slope [1/deg.]"])
    aoa_l0 = float(wing_dict["AoA @ L0 [deg.]"])
    cl_max = float(wing_dict["CL Max"])
    cd_min = float(wing_dict["CD Min"])

    return {
        "surface_area": surface_area,
        "a": dcl_daoa_slope,
        "C_L0": -dcl_daoa_slope * aoa_l0,
        "C_Lmax": cl_max,
        "C_Lmin": -cl_max,
        "C_D0": cd_min,
        "k": k,
        "C_Dfp": 1,
        "bias1": 1.0,
        "bias2": 0.5,
    }


def generate_fdm_input(wing_data: Dict[str, float]) -> str:
    """
    Looking at the new_fdm.f fortran code, this is the bare minimum for lift
    and drag tables.
    """

    template = """&aircraft_data
   aircraft%num_wings       = 2 ! M number of wings in aircraft
   aircraft%num_propellers  = 0
   aircraft%num_batteries   = 0
   aircraft%i_analysis_type = 0

!   Wings
!
!  WING: (1)    Component Name: rear_right_wing
   wing(1)%surface_area   = {surface_area}
   wing(1)%a   = {a}
   wing(1)%C_L0   = {C_L0}
   wing(1)%C_Lmax   = {C_Lmax}
   wing(1)%C_Lmin   = {C_Lmin}
   wing(1)%C_D0   = {C_D0}
   wing(1)%k   = {k}
   wing(1)%C_Dfp   = {C_Dfp}
!
!  WING: (2)    Component Name: rear_left_wing
   wing(2)%surface_area   = {surface_area}
   wing(2)%a   = {a}
   wing(2)%C_L0   = {C_L0}
   wing(2)%C_Lmax   = {C_Lmax}
   wing(2)%C_Lmin   = {C_Lmin}
   wing(2)%C_D0   = {C_D0}
   wing(2)%k   = {k}
   wing(2)%C_Dfp   = {C_Dfp}

/
"""
    return template.format(**wing_data)


def run_new_fdm(fdm_binary: str, fdm_input: str) -> str:
    with open('wing_analysis.inp', 'w') as file:
        file.write(fdm_input)

    cmd = "{} < wing_analysis.inp > wing_analysis.out".format(fdm_binary)
    status = os.system(cmd)
    if status == 2:
        raise KeyboardInterrupt

    with open('wing_analysis.out', 'r') as file:
        return file.read()


def parse_fdm_output(fdm_output: str, target_speed: float = 50.0) -> List[Dict[str, float]]:
    angles = dict()
    speeds = []

    state = ""
    for line in fdm_output.splitlines():
        line = line.rstrip()

        if line == "  Lift force (N)":
            state = "lift"
        elif line == "  Drag force (N)":
            state = "drag"
        elif line == "":
            state = ""

        if line.startswith("  Flight speed (m/s)"):
            speeds = [float(line[pos:pos+9]) for pos in range(20, 110, 9)]
        elif line.startswith("          ") and len(line) >= 110 and state:
            row = []
            for pos in range(11, 110, 9):
                try:
                    value = float(line[pos:pos+9])
                except ValueError:
                    value = math.nan
                row.append(value)

            if state == "lift":
                assert row[0] not in angles
                angles[row[0]] = [row[1:], None]
            elif state == "drag":
                assert row[0] in angles
                angles[row[0]][1] = row[1:]

    result = []
    for angle, val in angles.items():
        lifts, drags = val
        assert len(lifts) == len(speeds) and len(drags) == len(speeds)
        for speed, lift, drag in zip(speeds, lifts, drags):
            if speed != target_speed:
                continue

            # filter out bad datapoints
            if not math.isfinite(lift) or not math.isfinite(drag):
                continue

            # we have two identical wings, search for "call CLDwing" in new_fdm.f
            flight_load = 0.5 * math.sqrt(lift ** 2 + drag ** 2)

            result.append({
                "angle": angle,
                "speed": speed,
                "lift2x": lift,
                "drag2x": drag,
                "lift": 0.5 * lift,
                "drag": 0.5 * drag,
                "flight_load": flight_load,
            })

    return result


def calc_geom_data(profile: str, chord1: float, chord2: float,
                   span: float) -> Dict[str, float]:
    """ Copied from run_fd_calc.py for battery packing """

    match = NACA_PROFILE.match(profile)
    assert match
    thickness = float(match.group(3))

    root_chord = max([chord1, chord2])
    tip_chord = min([chord1, chord2])
    A = root_chord/2
    B = thickness/100*A
    C = tip_chord/2
    D = thickness/100*C

    available_volume = 1/6*span*(A*B+C*D+((A+C)*(B+D)))

    return {
        "available_volume": available_volume,
    }


def calc_weight_data(profile: str, chord1: float, chord2: float,
                     span: float, max_load: float) -> Dict[str, float]:
    """ Extracted from Creo models (Tools/Relations) """

    match = NACA_PROFILE.match(profile)
    assert match
    thickness = float(match.group(3))

    taper_offset = 0  # assuming no taper offset

    N = 1.5  # safety factor
    VE = 50  # max airspeed
    s = (chord1 + chord2) / 2 * span  # plan form area with taper accounted for
    ar = span ** 2 / s  # aspect ratio assuming non-constant chord wing
    sa = math.atan(abs(chord1 - chord2) * taper_offset / span)  # weep angle
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

    return {
        "max_load": max_load,
        "weight": weight,
    }


def combination_generator(
    profiles: Iterable[str],
    chords: Iterable[float],
    spans: Iterable[float],
    max_loads: Iterable[float],
) -> Generator[Dict[str, Any], None, None]:
    for profile in profiles:
        for chord in chords:
            for span in spans:
                for max_load in max_loads:
                    yield {
                        "profile": profile,
                        "chord": chord,
                        "span": span,
                        "max_load": max_load,
                    }


def datapoint_generator(
        fdm_binary: str,
        target_speed: float,
        combinations: Iterable[Dict[str, Any]]
) -> Generator[Dict[str, Any], None, None]:
    for comb in combinations:
        wing_data = calc_wing_data(
            comb["profile"], comb["chord"], comb["chord"], comb["span"])
        geom_data = calc_geom_data(
            comb["profile"], comb["chord"], comb["chord"], comb["span"])
        weight_data = calc_weight_data(
            comb["profile"], comb["chord"], comb["chord"], comb["span"],
            comb["max_load"])
        fdm_input = generate_fdm_input(wing_data)
        fdm_output = run_new_fdm(fdm_binary, fdm_input)
        lift_drags = parse_fdm_output(fdm_output, target_speed)
        for lift_drag in lift_drags:
            if lift_drag["flight_load"] < comb["max_load"]:
                yield {**comb, **lift_drag, **geom_data, **weight_data}


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output',
                        default='wing_analysis.csv',
                        help="output file name")
    parser.add_argument('--fdm',
                        default='new_fdm', metavar='PATH',
                        help="path to the new_fdm executable")
    parser.add_argument('--speed', type=float, default=50.0,
                        help='change the target speed')
    parser.add_argument('--naca', metavar='DDDD',
                        help='limits the search to this NACA number')
    parser.add_argument('--chord', type=float,
                        help='limits the search to this chord number')
    parser.add_argument('--span', type=float,
                        help='limits the search to this span number')
    parser.add_argument('--max-load', type=float,
                        help='limits the search to this max load number')
    parser.add_argument('--info', action='store_true',
                        help="print out all search ranges")
    args = parser.parse_args(args)

    if args.naca is not None:
        assert len(args.naca) == 4
        profiles = ["NACA " + args.naca]
    else:
        profiles = list(WINGS.keys())

    if args.chord is not None:
        chords = [args.chord]
    else:
        chords = range(500, 5500, 500)

    if args.span is not None:
        spans = [args.span]
    else:
        spans = range(1000, 11000, 1000)

    if args.max_load is not None:
        max_loads = [args.max_load]
    else:
        max_loads = range(1000, 11000, 1000)

    total = len(profiles) * len(chords) * len(spans) * len(max_loads) * 21
    if args.info:
        print("profiles:", ", ".join(profiles))
        print("chords:", ", ".join(map(str, chords)))
        print("spans:", ", ".join(map(str, spans)))
        print("max_loads:", ", ".join(map(str, max_loads)))
        print("target speed:", str(args.speed) + ", number of angles: 21")
        print("TOTAL:", total, "data points")
        return

    combinations = combination_generator(profiles, chords, spans, max_loads)
    datapoints = datapoint_generator(args.fdm, args.speed, combinations)

    with open(args.output, 'w', newline='') as file:
        row = next(datapoints)
        writer = csv.DictWriter(file, row.keys())
        writer.writeheader()
        writer.writerow(row)
        cnt = 0
        for row in datapoints:
            writer.writerow(row)
            if cnt % 1000 == 0:
                sys.stdout.write('\r')
                sys.stdout.write("{:10.2f}%".format(100 * cnt / total))
                sys.stdout.flush()
            cnt += 1


if __name__ == '__main__':
    run()
