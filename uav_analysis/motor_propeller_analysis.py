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

from typing import Any, Dict, Iterable, List, Optional

import csv
import math
import os
import sys

from .components import DATAPATH, MOTORS, PROPELLERS, get_bemp_data


def generate_fdm_input(
        bemp_comb: Dict[str, Any],
        voltage: float,
        propdata: str) -> str:
    template = """&aircraft_data
   aircraft%cname           = 'UAV'
   aircraft%ctype           = 'SymCPS UAV Design'
   aircraft%num_wings       = 0
   aircraft%mass            = 0
   aircraft%num_propellers  = 1
   aircraft%num_batteries   = 1
   aircraft%i_analysis_type = 0

!   Propeller(1 uses components named Prop_0, Motor_0, ESC_0
   propeller(1)%cname = '{prop_cname}'
   propeller(1)%ctype = 'MR'
   propeller(1)%prop_fname = '{prop_fname}'
   propeller(1)%radius = {prop_radius}
   propeller(1)%Ir = 10.0
   propeller(1)%motor_fname = '{motor_fname}'
   propeller(1)%KV = {motor_kv}
   propeller(1)%KT = {motor_kt}
   propeller(1)%I_max = {motor_i_max}
   propeller(1)%I_idle = {motor_i_idle}
   propeller(1)%maxpower = {motor_max_power}
   propeller(1)%Rw = {motor_rw}
   propeller(1)%icontrol = 1
   propeller(1)%ibattery = 1
   propeller(1)%spin = 1

!   Battery(1) is component named: Battery_0
   battery(1)%num_cells = 1
   battery(1)%voltage = {voltage}
   battery(1)%capacity = 0
   battery(1)%C_Continuous = 25
   battery(1)%C_Peak = 50
/
"""
    return template.format(
        prop_cname=bemp_comb["Propeller.Name"],
        prop_fname=os.path.join(
            propdata, bemp_comb["Propeller.Performance_File"]),
        prop_radius=float(bemp_comb["Propeller.Diameter_mm"]) * 0.5,
        motor_fname=bemp_comb["Motor.Name"],
        motor_kv=float(bemp_comb["Motor.KV [RPM/V]"]),
        motor_kt=float(bemp_comb["Motor.KT [Nm/A]"]),
        motor_i_max=float(bemp_comb["Motor.Max Current [A]"]),
        motor_i_idle=float(bemp_comb["Motor.Io Idle Current@10V [A]"]),
        motor_max_power=float(bemp_comb["Motor.Max Power [W]"]),
        motor_rw=float(bemp_comb["Motor.Internal Resistance [mOhm]"]) * 0.001,
        voltage=voltage,
    )


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
            if line.startswith("     Motor #"):
                assert line[129:142] == "r     Max Cur"
                continue
            elif line.startswith(" Max Volt  1"):
                linetype = "MaxVolt."
            elif line.startswith(" MV @40m/s 1"):
                linetype = "MaxVolt_at40."
            elif line.startswith(" Max Power 1"):
                linetype = "MaxPower."
            elif line.startswith(" Max Amps  1"):
                linetype = "MaxAmps."
            else:
                continue

            result[linetype + "OmegaRpm"] = float(line[25:38])
            result[linetype + "Voltage"] = float(line[38:51])
            result[linetype + "Thrust"] = float(line[51:64])
            result[linetype + "Torque"] = float(line[64:77])
            result[linetype + "Power"] = float(line[77:90])
            result[linetype + "Current"] = float(line[90:103])
            result[linetype + "MaxPower"] = float(line[116:129])
            result[linetype + "MaxCurrent"] = float(line[129:142])

        return result

    except ValueError as err:
        print("WARNING:", err)
        return None


def create_datapoint(combination: Dict[str, Any],
                     output_data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict()
    result["motor_name"] = combination["Motor.Name"]
    result["propeller_name"] = combination["Propeller.Name"]
    result['weight'] = (float(combination['Motor.Weight [grams]']) +
                        float(combination['Propeller.Weight_g'])) * 0.001
    result['propeller_diameter'] = float(combination["Propeller.Diameter_mm"]) * 0.001

    # result.update(output_data)

    result["voltage"] = output_data["MaxVolt.Voltage"]
    result["thrust"] = output_data["MaxVolt.Thrust"]
    result["power"] = output_data["MaxVolt.Power"]
    result["current"] = output_data["MaxVolt.Current"]

    result["thrust_at40"] = output_data["MaxVolt_at40.Thrust"]
    result["power_at40"] = output_data["MaxVolt_at40.Power"]
    result["current_at40"] = output_data["MaxVolt_at40.Current"]

    if output_data['MaxPower.Power'] < output_data['MaxAmps.Power']:
        result["max_voltage"] = output_data["MaxPower.Voltage"]
        result["max_thrust"] = output_data["MaxPower.Thrust"]
        result["max_power"] = output_data["MaxPower.Power"]
        result["max_current"] = output_data["MaxPower.Current"]
    else:
        result["max_voltage"] = output_data["MaxAmps.Voltage"]
        result["max_thrust"] = output_data["MaxAmps.Thrust"]
        result["max_power"] = output_data["MaxAmps.Power"]
        result["max_current"] = output_data["MaxAmps.Current"]

    result["thrust_per_weight"] = result["thrust"] / result["weight"]
    result["power_per_weight"] = result["power"] / result["weight"]
    result["thrust_per_power"] = result["thrust"] / result["power"]

    return result


def motor_propeller_generator(select_motor: Optional[str], select_propeller: Optional[str]):
    for motor in MOTORS:
        if select_motor and motor != select_motor:
            continue
        for propeller in PROPELLERS:
            if select_propeller and propeller != select_propeller:
                continue
            if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                continue

            yield get_bemp_data(None, motor, propeller)


def datapoint_generator(
    propeller: Optional[str],
    motor: Optional[str],
    voltages: List[float],
    fdm_binary: str,
    propdata: str,
):
    generator = motor_propeller_generator(
        select_propeller=propeller, select_motor=motor)
    voltages = sorted(voltages)

    for bemp_comb in generator:
        max_voltage = None
        for voltage in voltages:
            if max_voltage is not None and voltage < max_voltage * 0.25:
                continue

            fdm_input = generate_fdm_input(
                bemp_comb=bemp_comb,
                voltage=voltage,
                propdata=propdata)
            fdm_output = run_new_fdm(
                fdm_binary=fdm_binary,
                fdm_input=fdm_input)
            output_data = parse_fdm_output(
                fdm_output=fdm_output)

            if output_data is None:
                continue

            max_voltage = min(
                output_data["MaxPower.Voltage"], output_data["MaxAmps.Voltage"])

            if output_data["MaxVolt.OmegaRpm"] < 0.0:
                continue

            if output_data["MaxVolt.Voltage"] > max_voltage:
                break

            yield create_datapoint(bemp_comb, output_data)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=os.path.relpath(
                            os.path.join(DATAPATH, 'propeller')),
                        metavar='DIR', help="path to propeller data directory")
    parser.add_argument('--output', default='motor_propeller_analysis.csv',
                        metavar='FILENAME', help="output file name")
    parser.add_argument('--limit', type=int, metavar='NUM',
                        help="process only LIMIT number of combinations")
    parser.add_argument('--fdm', default='new_fdm', metavar='PATH',
                        help="path to fdm executable")
    parser.add_argument('--propeller', metavar='NAME',
                        help='limits the search space to this propeller')
    parser.add_argument('--motor', metavar='NAME',
                        help='limits the search space to this motor')
    parser.add_argument('--voltage', metavar='V', nargs="*", default=[
        200.0, 300.0, 400.0, 500.0, 600.0,
        700.0, 720.0, 740.0, 760.0, 780.0,
        800.0, 820.0, 840.0, 860.0, 880.0,
    ],
        help='limits the search space to these voltages')

    args = parser.parse_args(args)

    datapoints = datapoint_generator(
        propeller=args.propeller,
        motor=args.motor,
        voltages=args.voltage,
        fdm_binary=args.fdm,
        propdata=args.propdata)

    total = len(MOTORS) * len(PROPELLERS) * len(args.voltage)
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
