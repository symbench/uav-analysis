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

from typing import Any, Dict, Iterable, List, Optional, Generator

import concurrent.futures
import csv
import math
import os
import subprocess
import sys

from .components import DATAPATH, MOTORS, PROPELLERS, get_bemp_data


def generate_fdm_input(
        motor_prop: Dict[str, Any],
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
        prop_cname=motor_prop["Propeller.Name"],
        prop_fname=os.path.join(
            propdata, motor_prop["Propeller.Performance_File"]),
        prop_radius=float(motor_prop["Propeller.Diameter_mm"]) * 0.5,
        motor_fname=motor_prop["Motor.Name"],
        motor_kv=float(motor_prop["Motor.KV [RPM/V]"]),
        motor_kt=float(motor_prop["Motor.KT [Nm/A]"]),
        motor_i_max=float(motor_prop["Motor.Max Current [A]"]),
        motor_i_idle=float(motor_prop["Motor.Io Idle Current@10V [A]"]),
        motor_max_power=float(motor_prop["Motor.Max Power [W]"]),
        motor_rw=float(motor_prop["Motor.Internal Resistance [mOhm]"]) * 0.001,
        voltage=voltage,
    )


def run_new_fdm(fdm_binary: str, fdm_input: str) -> str:
    proc = subprocess.Popen(
        fdm_binary, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    fdm_output, _ = proc.communicate(input=fdm_input.encode())
    if proc.returncode != 0:
        raise KeyboardInterrupt
    return fdm_output.decode()


def parse_fdm_output(fdm_output: str) -> Optional[Dict[str, float]]:
    def parse(value: str) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(str(value) + " is not finite")
        return value

    try:
        result = dict()
        for line in fdm_output.splitlines():
            if line.startswith("     Motor #"):
                assert line[129:142] == "r     Max Cur"
                continue
            elif line.startswith(" Max Volt  1"):
                linetype = "MaxVolt."
            # elif line.startswith(" MV @40m/s 1"):
            #     linetype = "MaxVolt_at40."
            elif line.startswith(" Max Power 1"):
                linetype = "MaxPower."
            elif line.startswith(" Max Amps  1"):
                linetype = "MaxAmps."
            else:
                continue

            result[linetype + "OmegaRpm"] = parse(line[25:38])
            result[linetype + "Voltage"] = parse(line[38:51])
            result[linetype + "Thrust"] = parse(line[51:64])
            result[linetype + "Torque"] = parse(line[64:77])
            result[linetype + "Power"] = parse(line[77:90])
            result[linetype + "Current"] = parse(line[90:103])
            result[linetype + "MaxPower"] = parse(line[116:129])
            result[linetype + "MaxCurrent"] = parse(line[129:142])

        return result

    except ValueError as err:
        # print("WARNING:", err)
        return None


def create_datapoint(combination: Dict[str, Any],
                     output_data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict()
    result["motor_name"] = combination["Motor.Name"]
    result["propeller_name"] = combination["Propeller.Name"]
    result['weight'] = (float(combination['Motor.Weight [grams]']) +
                        float(combination['Propeller.Weight_g'])) * 0.001
    result['propeller_diameter'] = float(
        combination["Propeller.Diameter_mm"]) * 0.001

    # result.update(output_data)

    result["omega_rpm"] = output_data["MaxVolt.OmegaRpm"]
    result["voltage"] = output_data["MaxVolt.Voltage"]
    result["thrust"] = output_data["MaxVolt.Thrust"]
    result["torque"] = output_data["MaxVolt.Torque"]
    result["power"] = output_data["MaxVolt.Power"]
    result["current"] = output_data["MaxVolt.Current"]

    # result["thrust_at40"] = output_data["MaxVolt_at40.Thrust"]
    # result["power_at40"] = output_data["MaxVolt_at40.Power"]
    # result["current_at40"] = output_data["MaxVolt_at40.Current"]

    if output_data['MaxPower.Power'] < output_data['MaxAmps.Power']:
        result["max_omega_rpm"] = output_data["MaxPower.OmegaRpm"]
        result["max_voltage"] = output_data["MaxPower.Voltage"]
        result["max_thrust"] = output_data["MaxPower.Thrust"]
        result["max_torque"] = output_data["MaxPower.Torque"]
        result["max_power"] = output_data["MaxPower.Power"]
        result["max_current"] = output_data["MaxPower.Current"]
    else:
        result["max_omega_rpm"] = output_data["MaxAmps.OmegaRpm"]
        result["max_voltage"] = output_data["MaxAmps.Voltage"]
        result["max_thrust"] = output_data["MaxAmps.Thrust"]
        result["max_torque"] = output_data["MaxAmps.Torque"]
        result["max_power"] = output_data["MaxAmps.Power"]
        result["max_current"] = output_data["MaxAmps.Current"]

    result["thrust_per_weight"] = result["thrust"] / result["weight"]
    result["power_per_weight"] = result["power"] / result["weight"]
    result["thrust_per_power"] = result["thrust"] / result["power"]

    return result


def motor_prop_datapoints(
        voltages: List[float],
        fdm_binary: str,
        propdata: str,
        motor_prop: Dict[str, Any]) -> List[Dict[str, Any]]:
    result = []
    max_voltage = None
    for voltage in voltages:
        if max_voltage is not None and voltage < max_voltage * 0.25:
            continue

        fdm_input = generate_fdm_input(
            motor_prop=motor_prop,
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
        if voltage < max_voltage * 0.25:
            continue

        if output_data["MaxVolt.OmegaRpm"] < 0.0 or output_data["MaxVolt.Torque"] < 0.0:
            continue

        if output_data["MaxVolt.Voltage"] > max_voltage:
            break

        result.append(create_datapoint(motor_prop, output_data))
    return result


def motor_propeller_generator(select_motor: Optional[str],
                              select_propeller: Optional[str]) \
        -> Generator[Dict[str, Any], None, None]:
    for motor in MOTORS:
        if select_motor and motor != select_motor:
            continue
        for propeller in PROPELLERS:
            if select_propeller and propeller != select_propeller:
                continue
            if MOTORS[motor]['(A) Shaft Diameter [mm]'] > PROPELLERS[propeller]['Shaft_Diameter_mm']:
                continue

            yield get_bemp_data(None, motor, propeller)


def save_all_datapoints(
        iterable: Iterable[Dict[str, Any]],
        voltages: List[float],
        fdm_binary: str,
        propdata: str,
        output: str,
):
    voltages = sorted(voltages)

    def task(motor_prop: Dict[str, float]):
        # print(motor_prop)
        return motor_prop_datapoints(
            voltages=voltages,
            fdm_binary=fdm_binary,
            propdata=propdata,
            motor_prop=motor_prop)

    print("Generating datapoints:")
    with open(output, 'w', newline='') as file:
        writer = None
        count = 0

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            iterable = executor.map(task, iterable, chunksize=10)
            for rows in iterable:
                for row in rows:
                    if writer is None:
                        writer = csv.DictWriter(file, row.keys())
                        writer.writeheader()
                    writer.writerow(row)
                    if count % 100 == 0:
                        sys.stdout.write("\r{}".format(count))
                        sys.stdout.flush()
                    count += 1

    print("\nResults saved to", output)


def run_single(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=os.path.relpath(
                            os.path.join(DATAPATH, 'propeller')),
                        metavar='DIR', help="path to propeller data directory")
    parser.add_argument('--fdm', default='new_fdm', metavar='PATH',
                        help="path to fdm executable")
    parser.add_argument('motor', metavar='MOTOR', help='motor name')
    parser.add_argument('propeller', metavar='PROPELLER',
                        help='propeller name')
    parser.add_argument('voltage', metavar='VOLTAGE',
                        type=float, help='voltage level')

    args = parser.parse_args(args)

    motor_prop = get_bemp_data(None, args.motor, args.propeller)

    fdm_input = generate_fdm_input(motor_prop, args.voltage, args.propdata)
    with open("motor_propeller_single.inp", 'w') as file:
        file.write(fdm_input)

    fdm_output = run_new_fdm(
        fdm_binary=args.fdm,
        fdm_input=fdm_input)
    with open("motor_propeller_single.out", 'w') as file:
        file.write(fdm_output)

    print(fdm_output)

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
    parser.add_argument('--fdm', default='new_fdm', metavar='PATH',
                        help="path to fdm executable")
    parser.add_argument('--propeller', metavar='NAME',
                        help='limits the search space to this propeller')
    parser.add_argument('--motor', metavar='NAME',
                        help='limits the search space to this motor')
    parser.add_argument('--voltage', metavar='V', nargs="*", default=[
        0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0,
        10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0, 40.0, 50.0, 80.0,
        100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0,
        500.0, 550.0, 600.0, 650.0, 700.0, 750.0, 800.0, 850.0, 900.0,
        1000.0, 1100.0, 1200.0, 1300.0, 1400.0, 1500.0
    ],
        help='limits the search space to these voltages')

    args = parser.parse_args(args)

    save_all_datapoints(
        iterable=motor_propeller_generator(
            select_propeller=args.propeller,
            select_motor=args.motor),
        voltages=args.voltage,
        fdm_binary=args.fdm,
        propdata=args.propdata,
        output=args.output)


if __name__ == '__main__':
    run()
