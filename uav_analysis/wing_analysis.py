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

from typing import Any, List, Dict, Union, Generator, Iterable

import sys
import csv
import math
import os
import re
import sympy

from .components import DATAPATH, AERO_INFO


NACA_PROFILE = re.compile(r"^NACA \d\d(\d\d)$")


def get_wing_thickness(naca_profile: str) -> int:
    match = NACA_PROFILE.match(naca_profile)
    assert match
    return int(match.group(1))


def get_wing_data(naca_profile: str,
                  chord1: Union[float, sympy.Expr],  # m
                  chord2: Union[float, sympy.Expr],  # m
                  span: Union[float, sympy.Expr],    # m
                  ) -> Dict[str, Union[float, sympy.Expr]]:
    """ Copied from run_fd_calc.py """
    match = NACA_PROFILE.match(naca_profile)
    assert match
    wing_dict = AERO_INFO[naca_profile]

    MC = (chord1 + chord2) / 2  # Mean chord
    SA = MC * span  # Surface area = planform area
    TR = sympy.Min(chord1, chord2) / sympy.Max(chord1, chord2)  # Taper ratio
    AR = span ** 2 / SA  # aspect ratio, modified defintion for tapered wings
    Hfun = 0.0524 * TR ** 4 - 0.15 * TR ** 3 + \
        0.1659 * TR ** 2 - 0.0706 * TR + 0.0119
    k = (1 + Hfun * AR) / (math.pi * AR)

    dcl_daoa_slope = float(wing_dict["dCl_dAoA_Slope"])
    aoa_l0 = float(wing_dict["AoA_L0"])
    cl_max = float(wing_dict["CL_Max"])
    cd_min = float(wing_dict["CD_Min"])

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


def get_wing_weight(thickness: float,
                    chord1: Union[float, sympy.Expr],    # m
                    chord2: Union[float, sympy.Expr],    # m
                    span: Union[float, sympy.Expr],      # m
                    max_load: Union[float, sympy.Expr],  # N
                    ) -> Union[float, sympy.Expr]:       # kg
    chord1 = chord1 * 1000
    chord2 = chord2 * 1000
    span = span * 1000
    root_chord = sympy.Max(chord1, chord2)
    tip_chord = sympy.Min(chord1, chord2)
    TR = tip_chord / root_chord
    MC = (chord1+chord2)/2

    """ Extracted from Creo models (Tools/Relations) """
    real_thick = (thickness/100)*MC
    rho = 1329.53246
    yield_strength = 4.413e+9 * 0.8 / 1.5
    skin_thick = 0.15 / 1000
    meter_width = real_thick/1000
    meter_chord = MC/1000
    meter_span = span/1000
    S = meter_span*meter_chord
    TC = thickness/100

    max_moment = ((max_load / meter_span) * meter_span ** 2) / 2

    t_beam = 1/2*(meter_width - ((yield_strength*meter_width*(yield_strength *
                  meter_width ** 3 - 6 * max_moment)) ** (1/4) / yield_strength ** 0.5))

    volume = (meter_width ** 2 - (meter_width - 2 * t_beam) ** 2) * meter_span
    Mspar = volume*rho
    Mrib = (((3.1 * 1.37 * S * 0.5) * (meter_chord * TC) ** 0.5) / (1 + TR)) * \
        ((1 + TR + TR ** 2) + 1.1 * meter_chord*TC*(1 + TR+TR ** 2 + TR ** 3))
    Mskin = 2*S*skin_thick*rho
    Mwing = Mspar + Mskin + Mrib

    return Mwing


def get_wing_volume(naca_profile: str,
                    chord1: float,  # m
                    chord2: float,  # m
                    span: float,    # m
                    ) -> float:     # m^3
    """ Copied from run_fd_calc.py for battery packing """
    thickness = get_wing_thickness(naca_profile)

    # it does not matter, which one is which
    root_chord = chord1
    tip_chord = chord2
    A = root_chord/2
    B = thickness/100*A
    C = tip_chord/2
    D = thickness/100*C

    available_volume = 1/6*span*(A*B+C*D+((A+C)*(B+D)))

    return available_volume


def generate_fdm_input(wing_data: Dict[str, float],
                       max_load: float,
                       ) -> str:
    """
    Looking at the new_fdm.f fortran code, this is the bare minimum for lift
    and drag tables.
    """

    template = """&aircraft_data
   aircraft%num_wings       = 2
   aircraft%num_propellers  = 0
   aircraft%num_batteries   = 0
   aircraft%i_analysis_type = 0

!  WING: (1)    Component Name: rear_right_wing
   wing(1)%surface_area   = {surface_area}
   wing(1)%a   = {a}
   wing(1)%C_L0   = {C_L0}
   wing(1)%C_Lmax   = {C_Lmax}
   wing(1)%C_Lmin   = {C_Lmin}
   wing(1)%C_D0   = {C_D0}
   wing(1)%k   = {k}
   wing(1)%C_Dfp   = {C_Dfp}
   wing(1)%max_load  = {max_load}
   wing(1)%bias1   =  {bias1}
   wing(1)%bias2   =  {bias2}
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
   wing(2)%max_load  = {max_load}
   wing(2)%bias1   =  {bias1}
   wing(2)%bias2   =  {bias2}

/
"""
    return template.format(
        surface_area=wing_data["surface_area"] * 1e6,
        a=wing_data["a"],
        C_L0=wing_data["C_L0"],
        C_Lmax=wing_data["C_Lmax"],
        C_Lmin=wing_data["C_Lmin"],
        C_D0=wing_data["C_D0"],
        k=wing_data["k"],
        C_Dfp=wing_data["C_Dfp"],
        bias1=wing_data["bias1"],
        bias2=wing_data["bias2"],
        max_load=max_load,
    )


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
                        "chord": chord,         # mm
                        "span": span,           # mm
                        "max_load": max_load,   # N
                    }


def datapoint_generator(
        fdm_binary: str,
        target_speed: float,
        load_safety: float,
        combinations: Iterable[Dict[str, Any]]
) -> Generator[Dict[str, Any], None, None]:
    for comb in combinations:
        thickness = get_wing_thickness(comb["profile"])
        wing_data = get_wing_data(
            comb["profile"],
            comb["chord"],
            comb["chord"],
            comb["span"])
        wing_weight = get_wing_weight(
            thickness,
            comb["chord"],
            comb["chord"],
            comb["span"],
            comb["max_load"])
        wing_volume = get_wing_volume(
            comb["profile"],
            comb["chord"],
            comb["chord"],
            comb["span"])
        fdm_input = generate_fdm_input(wing_data, max_load=comb["max_load"])
        fdm_output = run_new_fdm(fdm_binary, fdm_input)
        lift_drags = parse_fdm_output(fdm_output, target_speed)
        for lift_drag in lift_drags:
            if lift_drag["flight_load"] * load_safety < comb["max_load"]:
                lift_per_drag = lift_drag["lift"] / lift_drag["drag"]
                yield {**comb,
                       **lift_drag,
                       "wing_weight": wing_weight,
                       "wing_volume": wing_volume,
                       "lift_per_drag": lift_per_drag,
                       }


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output',
                        default='wing_analysis.csv',
                        help="output file name")
    parser.add_argument('--fdm',
                        default=os.path.relpath(os.path.join(
                            DATAPATH, '..', '..', 'flight-dynamics-model', 'bin', 'new_fdm_step0')),
                        metavar='PATH', help="path to fdm executable")
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
    parser.add_argument('--load-safety', type=float, default=1.25,
                        help='safety factor for maximum vs flight loads')
    parser.add_argument('--info', action='store_true',
                        help="print out all search ranges")
    args = parser.parse_args(args)

    if args.naca is not None:
        assert len(args.naca) == 4
        profiles = ["NACA " + args.naca]
    else:
        profiles = list(AERO_INFO.keys())

    if args.chord is not None:
        chords = [args.chord]
    else:
        chords = [cm * 0.01 for cm in range(5, 55, 5)]

    if args.span is not None:
        spans = [args.span]
    else:
        spans = [cm * 0.01 for cm in range(20, 160, 10)]

    if args.max_load is not None:
        max_loads = [args.max_load]
    else:
        max_loads = [n for n in range(30, 110, 10)]

    if args.info:
        print("profiles:", ", ".join(profiles))
        print("chords:", ", ".join(map(str, chords)))
        print("spans:", ", ".join(map(str, spans)))
        print("max_loads:", ", ".join(map(str, max_loads)))
        print("target speed:", str(args.speed))
        print("number of angles: 21")
        return

    combinations = combination_generator(profiles, chords, spans, max_loads)
    datapoints = datapoint_generator(
        fdm_binary=args.fdm,
        target_speed=args.speed,
        load_safety=args.load_safety,
        combinations=combinations)

    with open(args.output, 'w', newline='') as file:
        writer = None
        count = 0

        for row in datapoints:
            if writer is None:
                writer = csv.DictWriter(file, row.keys())
                writer.writeheader()
            writer.writerow(row)
            if count % 1000 == 0:
                sys.stdout.write("\rGenerated: {}".format(count))
                sys.stdout.flush()
            count += 1

    sys.stdout.write("\rGenerated: {}".format(count))
    print("\nResults saved to", args.output)


if __name__ == '__main__':
    run()
