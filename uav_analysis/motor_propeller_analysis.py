#!/usr/bin/env python3
# Copyright (C) 2021, Carlos Olea and Miklos Maroti
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

from typing import Any, Dict, Iterable, Optional

import csv
import math
import os
import sys

from .components import MOTORS, PROPELLERS, motor_propeller_generator


def generate_fdm_input(bemp_comb: Dict[str, Any], propdata: str) -> str:
    str_return = "&aircraft_data\n"
    str_return += "   aircraft%cname           = 'UAV'\n"
    str_return += "   aircraft%ctype           = 'SymCPS UAV Design'\n"
    str_return += "   aircraft%num_wings       = 0\n"
    str_return += "   aircraft%mass            = 0\n"
    str_return += "   aircraft%num_propellers  = 1\n"
    str_return += "   aircraft%num_batteries   = 1\n"
    str_return += "   aircraft%i_analysis_type = 0\n"
    str_return += "\n"
    str_return += "!   Propeller(1 uses components named Prop_0, Motor_0, ESC_0\n"
    str_return += "   propeller(1)%cname = '{}'\n".format(
        bemp_comb["Propeller.Name"])
    str_return += "   propeller(1)%ctype = 'MR'\n"
    str_return += "   propeller(1)%prop_fname = '{}'\n".format(
        os.path.join(propdata, bemp_comb["Propeller.Performance_File"]))
    str_return += "   propeller(1)%radius = {}\n".format(
        float(bemp_comb["Propeller.Diameter_mm"]) / 2.0)
    # unknown value but not used
    str_return += "   propeller(1)%Ir = {}\n".format(10.0)
    # not used
    str_return += "   propeller(1)%motor_fname = '{}'\n".format(
        bemp_comb["Motor.Name"])
    str_return += "   propeller(1)%KV = {}\n".format(
        bemp_comb["Motor.KV [RPM/V]"])
    str_return += "   propeller(1)%KT = {}\n".format(
        bemp_comb["Motor.KT [Nm/A]"])
    str_return += "   propeller(1)%I_max = {}\n".format(
        bemp_comb["Motor.Max Current [A]"])
    str_return += "   propeller(1)%I_idle = {}\n".format(
        bemp_comb["Motor.Io Idle Current@10V [A]"])
    str_return += "   propeller(1)%maxpower = {}\n".format(
        bemp_comb["Motor.Max Power [W]"])
    str_return += "   propeller(1)%Rw = {}\n".format(
        float(bemp_comb["Motor.Internal Resistance [mOhm]"]) / 1000.0)
    str_return += "   propeller(1)%icontrol = 1\n"
    str_return += "   propeller(1)%ibattery = 1\n"
    str_return += "   propeller(1)%spin = 1\n"
    str_return += "\n"
    str_return += "!   Battery(1) is component named: Battery_0\n"
    str_return += "   battery(1)%num_cells = 0\n"
    str_return += "   battery(1)%voltage = 0\n"
    str_return += "   battery(1)%capacity = 0\n"
    str_return += "   battery(1)%C_Continuous = 0\n"
    str_return += "   battery(1)%C_Peak = 0\n"
    str_return += "/\n"

    return str_return


def run_new_fdm(fdm_binary: str, fdm_input: str) -> str:
    with open('motor_propeller_analysis.inp', 'w') as file:
        file.write(fdm_input)

    cmd = "{} < motor_propeller_analysis.inp > motor_propeller_analysis.out".format(
        fdm_binary)
    status = os.system(cmd)
    if status == 2:
        raise KeyboardInterrupt

    with open('motor_propeller_analysis.out', 'r') as file:
        return file.read()


def parse_fdm_output(fdm_output: str) -> Optional[Dict[str, float]]:
    try:
        result = dict()
        for line in fdm_output.splitlines():
            if line.startswith(" Max Power 1"):
                linetype = "MaxPower."
            elif line.startswith(" Max Amps  1"):
                linetype = "MaxAmps."
            else:
                continue

            result[linetype + "OmegaRpm"] = float(line[22:32])
            result[linetype + "Voltage"] = float(line[32:42])
            result[linetype + "Thrust"] = float(line[42:52])
            result[linetype + "Torque"] = float(line[52:62])
            result[linetype + "Power"] = float(line[62:72])
            result[linetype + "Current"] = float(line[72:82])
            result[linetype + "MaxPower"] = float(line[92:102])
            result[linetype + "MaxCurrent"] = float(line[102:112])

        assert 'MaxPower.Voltage' in result and 'MaxAmps.Voltage' in result
        return result

    except:
        return None


def create_datapoint(combination: Dict[str, Any],
                     output_data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict()
    result["Motor.Name"] = combination["Motor.Name"]
    result["Motor.Weight"] = float(combination['Motor.Weight [grams]']) / 1000
    result["Propeller.Name"] = combination["Propeller.Name"]
    result["Propeller.Weight"] = float(
        combination['Propeller.Weight_g']) / 1000

    result.update(output_data)

    result['Combined.Weight'] = result['Motor.Weight'] + \
        result['Propeller.Weight']

    if result['MaxPower.Power'] < result['MaxAmps.Power']:
        prefix = "MaxPower."
    else:
        prefix = "MaxAmps."

    for name in ["OmegaRpm", "Voltage", "Thrust", "Torque", "Power", "Current"]:
        result["Combined.Max" + name] = result[prefix + name]

    result["Combined.MaxThrustPerWeight"] = result["Combined.MaxThrust"] / \
        result["Combined.Weight"]
    result["Combined.MaxPowerPerWeight"] = result["Combined.MaxPower"] / \
        result["Combined.Weight"]

    return result


def datapoint_generator(
    propeller: Optional[str],
    motor: Optional[str],
    fdm_binary: str,
    propdata: str,
):
    generator = motor_propeller_generator()

    for combination in generator:
        if propeller and propeller != combination["Propeller.Name"]:
            continue
        if motor and motor != combination["Motor.Name"]:
            continue

        fdm_input = generate_fdm_input(combination, propdata)
        fdm_output = run_new_fdm(fdm_binary, fdm_input)
        output_data = parse_fdm_output(fdm_output)
        if output_data is None:
            continue

        yield create_datapoint(combination, output_data)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=os.path.relpath(os.path.join(
                            os.path.dirname(__file__), 'data_hackathon2_uam', 'propeller')),
                        type=str,
                        metavar='DIR',
                        help="path to propeller data directory")
    parser.add_argument('--output',
                        default='motor_propeller_analysis.csv',
                        type=str,
                        metavar='FILENAME',
                        help="output file name")
    parser.add_argument('--limit',
                        default=None,
                        type=int,
                        metavar='NUM',
                        help="process only LIMIT number of combinations")
    parser.add_argument('--fdm',
                        default='new_fdm',
                        type=str,
                        metavar='PATH',
                        help="path to fdm executable")
    parser.add_argument('--propeller',
                        metavar='NAME',
                        help='limits the search space to this propeller')
    parser.add_argument('--motor',
                        metavar='NAME',
                        help='limits the search space to this motor')

    args = parser.parse_args(args)

    datapoints = datapoint_generator(
        propeller=args.propeller,
        motor=args.motor,
        fdm_binary=args.fdm,
        propdata=args.propdata)

    total = len(MOTORS) * len(PROPELLERS)
    print("Progress: ")

    with open(args.output, 'w', newline='') as file:
        row = next(datapoints)
        writer = csv.DictWriter(file, row.keys())
        writer.writeheader()
        writer.writerow(row)
        cnt = 0
        for row in datapoints:
            writer.writerow(row)
            if cnt % 100 == 0:
                sys.stdout.write("\r{:6.2f}%".format(100 * cnt / total))
                sys.stdout.flush()
            cnt += 1
            if args.limit and cnt >= args.limit:
                break

        print("\nResults saved to", args.output)


if __name__ == '__main__':
    run()
