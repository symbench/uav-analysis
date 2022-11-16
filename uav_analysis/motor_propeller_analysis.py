#!/usr/bin/env python3
# Copyright (C) 2021-2022, Carlos Olea and Miklos Maroti
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

from typing import Any, Dict, Iterable, List, Optional, Tuple, Generator

import concurrent.futures
import csv
import math
import os
import subprocess
import sys

from .components import MOTORS, PROPELLERS, FDM_PATH, PROP_PATH


def generate_fdm_input(
        motor: Dict[str, Any],
        propeller: Dict[str, Any],
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

!  Propeller(1) uses components named Prop_0, Motor_0, ESC_0
   propeller(1)%cname = '{prop_cname}'
   propeller(1)%ctype = 'MR'
   propeller(1)%prop_fname = '{prop_fname}'
   propeller(1)%radius = {prop_radius}
   propeller(1)%spin = 1
   propeller(1)%Ir = {prop_ir}
   propeller(1)%motor_fname = '{motor_fname}'
   propeller(1)%Rw = {motor_rw}
   propeller(1)%KV = {motor_kv}
   propeller(1)%KT = {motor_kt}
   propeller(1)%I_max = {motor_i_max}
   propeller(1)%I_idle = {motor_i_idle}
   propeller(1)%maxpower = {motor_max_power}
   propeller(1)%icontrol = 1
   propeller(1)%ibattery = 1

!  Battery(1) is component named: Battery_0
   battery(1)%num_cells = 1
   battery(1)%voltage = {voltage}
   battery(1)%capacity = 0
   battery(1)%C_Continuous = 25
   battery(1)%C_Peak = 50

!  Controls
   control%requested_lateral_speed = 20
/
"""
    return template.format(
        prop_cname=propeller["MODEL_NAME"],
        prop_fname=os.path.join(
            propdata, propeller["Performance_File"]),
        prop_radius=float(propeller["DIAMETER"]) * 0.5,  # mm
        prop_ir=float(propeller["WEIGHT"]) * \
        float(propeller["DIAMETER"]) ** 2 / 12.0,  # kg mm^2
        motor_fname=motor["MODEL_NAME"],
        motor_kv=float(motor["KV"]),  # RPM/V
        motor_kt=float(motor["KT"]),  # Nm/A
        motor_i_max=float(motor["MAX_CURRENT"]),  # A
        motor_i_idle=float(motor["IO_IDLE_CURRENT_10V"]),  # A
        motor_max_power=float(motor["MAX_POWER"]),  # W
        motor_rw=float(motor["INTERNAL_RESISTANCE"]) * 0.001,  # Ohm
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
            elif line.startswith(" MaxAt20.0 1"):
                linetype = "MaxAt20."
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


def create_single_datapoint(motor: Dict[str, Any],
                            propeller: Dict[str, Any],
                            output_data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict()
    result["motor_name"] = motor["MODEL_NAME"]
    result["propeller_name"] = propeller["MODEL_NAME"]
    result['weight'] = float(motor["WEIGHT"]) + \
        float(propeller['WEIGHT'])  # kg
    result['propeller_diameter'] = float(propeller["DIAMETER"]) * 0.001  # m
    result['propeller_rpm_max'] = float(propeller["RPM_MAX"])  # rpm
    result['propeller_rpm_min'] = float(propeller["RPM_MIN"])  # rpm
    result['propeller_j_max'] = float(propeller["J_MAX"])      # 1

    result["omega_rpm"] = output_data["MaxVolt.OmegaRpm"]  # rpm
    result["voltage"] = output_data["MaxVolt.Voltage"]     # V
    result["thrust"] = output_data["MaxVolt.Thrust"]       # N
    result["power"] = output_data["MaxVolt.Power"]         # W
    result["current"] = output_data["MaxVolt.Current"]     # A

    result["omega_rpm_at20"] = output_data.get("MaxAt20.OmegaRpm", math.nan)
    result["thrust_at20"] = output_data.get("MaxAt20.Thrust", math.nan)
    result["power_at20"] = output_data.get("MaxAt20.Power", math.nan)
    result["current_at20"] = output_data.get("MaxAt20.Current", math.nan)
    result["j_at20"] = 20.0 / \
        (result["omega_rpm_at20"] / 60.0 * result['propeller_diameter'])

    # the other values (the thrust) are not reliable
    if output_data["MaxPower.Voltage"] < output_data["MaxAmps.Voltage"]:
        result["max_omega_rpm"] = output_data["MaxPower.OmegaRpm"]
        result["max_voltage"] = output_data["MaxPower.Voltage"]
        result["max_thrust"] = output_data["MaxPower.Thrust"]
        result["max_power"] = output_data["MaxPower.Power"]
        result["max_current"] = output_data["MaxPower.Current"]
    else:
        result["max_omega_rpm"] = output_data["MaxAmps.OmegaRpm"]
        result["max_voltage"] = output_data["MaxAmps.Voltage"]
        result["max_thrust"] = output_data["MaxAmps.Thrust"]
        result["max_power"] = output_data["MaxAmps.Power"]
        result["max_current"] = output_data["MaxAmps.Current"]

    result["thrust_per_weight"] = result["thrust"] / result["weight"]
    result["power_per_weight"] = result["power"] / result["weight"]
    result["thrust_per_power"] = result["thrust"] / result["power"]

    result["thrust_per_weight_at20"] = result["thrust_at20"] / result["weight"]
    result["power_per_weight_at20"] = result["power_at20"] / result["weight"]
    result["thrust_per_power_at20"] = result["thrust_at20"] / \
        result["power_at20"] if result["power_at20"] != 0 else math.nan

    return result


def generate_multi_datapoints(
        voltages: List[float],
        fdm_binary: str,
        propdata: str,
        rpm_limits: bool,
        at20_limits: bool,
        motor: Dict[str, Any],
        propeller: Dict[str, Any]) -> List[Dict[str, Any]]:
    result = []
    for voltage in voltages:
        fdm_input = generate_fdm_input(
            motor=motor,
            propeller=propeller,
            voltage=voltage,
            propdata=propdata)
        fdm_output = run_new_fdm(
            fdm_binary=fdm_binary,
            fdm_input=fdm_input)
        output_data = parse_fdm_output(
            fdm_output=fdm_output)

        if output_data is None:
            continue

        datapoint = create_single_datapoint(motor, propeller, output_data)

        if datapoint["max_voltage"] < voltage:
            break

        if rpm_limits:
            if datapoint["omega_rpm"] > datapoint["propeller_rpm_max"]:
                break

            if datapoint["omega_rpm_at20"] > datapoint["propeller_rpm_max"]:
                break

            if datapoint["omega_rpm"] < datapoint["propeller_rpm_min"]:
                continue

        if at20_limits:
            if datapoint["j_at20"] > datapoint["propeller_j_max"]:
                continue

            if datapoint["thrust_at20"] <= 0:
                continue

        if min(datapoint["omega_rpm"], datapoint["voltage"],
               datapoint["power"], datapoint["thrust"],
               datapoint["current"]) <= 0.0:
            continue

        result.append(datapoint)

    return result


def motor_propeller_generator(select_motor: Optional[str], select_propeller: Optional[str]) \
        -> Generator[Tuple[Dict[str, Any], Dict[str, Any]], None, None]:
    for motor in MOTORS:
        if select_motor and motor != select_motor:
            continue
        if float(MOTORS[motor]['MAX_POWER']) <= 0.0:
            print("WARNING: invalid MAX_POWER for", motor)
            continue

        for propeller in PROPELLERS:
            if select_propeller and propeller != select_propeller:
                continue
            if MOTORS[motor]['SHAFT_DIAMETER'] > PROPELLERS[propeller]['SHAFT_DIAMETER']:
                continue

            yield (MOTORS[motor], PROPELLERS[propeller])


def save_all_datapoints(
        iterable: Iterable[Tuple[Dict[str, Any], Dict[str, Any]]],
        voltages: List[float],
        fdm_binary: str,
        propdata: str,
        output: str,
        rpm_limits: bool,
        at20_limits: bool):
    voltages = sorted(voltages)

    def task(motor_prop):
        # return [{"motor": motor_prop[0], "propeller": motor_prop[1]}]
        return generate_multi_datapoints(
            voltages=voltages,
            fdm_binary=fdm_binary,
            propdata=propdata,
            rpm_limits=rpm_limits,
            at20_limits=at20_limits,
            motor=motor_prop[0],
            propeller=motor_prop[1])

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
                        sys.stdout.write("\rGenerated: {}".format(count))
                        sys.stdout.flush()
                    count += 1

    sys.stdout.write("\rGenerated: {}".format(count))
    print("\nResults saved to", output)


def run_single(args=None):
    import argparse
    import json

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=PROP_PATH,
                        metavar='DIR', help="path to propeller data directory")
    parser.add_argument('--fdm',
                        default=os.path.join(FDM_PATH, 'new_fdm_step0'),
                        metavar='PATH', help="path to fdm executable")
    parser.add_argument('motor', metavar='MOTOR', help='motor name')
    parser.add_argument('propeller', metavar='PROPELLER',
                        help='propeller name')
    parser.add_argument('voltage', metavar='VOLTAGE',
                        type=float, help='voltage level')

    args = parser.parse_args(args)

    if args.motor not in MOTORS:
        raise ValueError("invalid motor name")
    motor = MOTORS[args.motor]

    if args.propeller not in PROPELLERS:
        raise ValueError("invalid propeller name")
    propeller = PROPELLERS[args.propeller]

    fdm_input = generate_fdm_input(
        motor, propeller, args.voltage, args.propdata)
    with open("motor_propeller_single.inp", 'w') as file:
        file.write(fdm_input)

    fdm_output = run_new_fdm(
        fdm_binary=args.fdm,
        fdm_input=fdm_input)
    with open("motor_propeller_single.out", 'w') as file:
        file.write(fdm_output)

    print(fdm_output)

    output_data = parse_fdm_output(fdm_output)
    datapoint = create_single_datapoint(motor, propeller, output_data)

    print(json.dumps(datapoint, indent=2))


def approximate_motor_propeller(motor: str,
                                propeller: str,
                                propdata: str,
                                fdm: str,
                                ) -> Dict[str, Any]:
    motor = MOTORS[motor]
    propeller = PROPELLERS[propeller]

    motor_rw = float(motor["INTERNAL_RESISTANCE"]) * 0.001  # Ohm
    motor_i_idle = float(motor["IO_IDLE_CURRENT_10V"])      # A
    min_voltage = motor_rw * motor_i_idle + 0.01  # guard value

    fdm_input = generate_fdm_input(
        motor, propeller, min_voltage, propdata)
    fdm_output = run_new_fdm(
        fdm_binary=fdm,
        fdm_input=fdm_input)
    output_data = parse_fdm_output(fdm_output)
    min_datapoint = create_single_datapoint(motor, propeller, output_data)

    max_voltage = min_datapoint["max_voltage"] - 0.01  # guard value
    med_voltage = (min_voltage + max_voltage) * 0.5

    fdm_input = generate_fdm_input(
        motor, propeller, med_voltage, propdata)
    fdm_output = run_new_fdm(
        fdm_binary=fdm,
        fdm_input=fdm_input)
    output_data = parse_fdm_output(fdm_output)
    med_datapoint = create_single_datapoint(motor, propeller, output_data)

    fdm_input = generate_fdm_input(
        motor, propeller, max_voltage, propdata)
    fdm_output = run_new_fdm(
        fdm_binary=fdm,
        fdm_input=fdm_input)
    output_data = parse_fdm_output(fdm_output)
    max_datapoint = create_single_datapoint(motor, propeller, output_data)
    # print(fdm_output)

    return {
        "motor_name": min_datapoint["motor_name"],
        "propeller_name": min_datapoint["propeller_name"],
        "weight": min_datapoint["weight"],
        "min_voltage": min_voltage,
        "min_omega_rpm": min_datapoint["omega_rpm"],
        "min_thrust": min_datapoint["thrust"],
        "min_power": min_datapoint["power"],
        "min_current": min_datapoint["current"],
        "min_omega_rpm_at20": min_datapoint["omega_rpm_at20"],
        "min_thrust_at20": min_datapoint["thrust_at20"],
        "min_power_at20": min_datapoint["power_at20"],
        "min_current_at20": min_datapoint["current_at20"],
        "med_voltage": med_voltage,
        "med_omega_rpm": med_datapoint["omega_rpm"],
        "med_thrust": med_datapoint["thrust"],
        "med_power": med_datapoint["power"],
        "med_current": med_datapoint["current"],
        "med_omega_rpm_at20": med_datapoint["omega_rpm_at20"],
        "med_thrust_at20": med_datapoint["thrust_at20"],
        "med_power_at20": med_datapoint["power_at20"],
        "med_current_at20": med_datapoint["current_at20"],
        "max_voltage": max_voltage,
        "max_omega_rpm": max_datapoint["omega_rpm"],
        "max_thrust": max_datapoint["thrust"],
        "max_power": max_datapoint["power"],
        "max_current": max_datapoint["current"],
        "max_omega_rpm_at20": max_datapoint["omega_rpm_at20"],
        "max_thrust_at20": max_datapoint["thrust_at20"],
        "max_power_at20": max_datapoint["power_at20"],
        "max_current_at20": max_datapoint["current_at20"],
    }


def run_approximate(args=None):
    import argparse
    import json

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=PROP_PATH,
                        metavar='DIR', help="path to propeller data directory")
    parser.add_argument('--fdm',
                        default=os.path.join(FDM_PATH, 'new_fdm_step0'),
                        metavar='PATH', help="path to fdm executable")
    parser.add_argument('motor', metavar='MOTOR', help='motor name')
    parser.add_argument('propeller', metavar='PROPELLER',
                        help='propeller name')

    args = parser.parse_args(args)

    if args.motor not in MOTORS:
        raise ValueError("invalid motor name")

    if args.propeller not in PROPELLERS:
        raise ValueError("invalid propeller name")

    data = approximate_motor_propeller(
        args.motor, args.propeller, args.propdata, args.fdm)
    print(json.dumps(data, indent=2))


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=PROP_PATH,
                        metavar='DIR', help="path to propeller data directory")
    parser.add_argument('--output', default='motor_propeller_analysis.csv',
                        metavar='FILENAME', help="output file name")
    parser.add_argument('--no-rpm-limits', action='store_true',
                        help="do not use propeller table min/max RPM limits")
    parser.add_argument('--no-at20-limits', action='store_true',
                        help="do not check 20 m/s power and J values")
    parser.add_argument('--fdm',
                        default=os.path.join(FDM_PATH, 'new_fdm_step0'),
                        metavar='PATH', help="path to fdm executable")
    parser.add_argument('--propeller', metavar='NAME',
                        help='limits the search space to this propeller')
    parser.add_argument('--motor', metavar='NAME',
                        help='limits the search space to this motor')
    parser.add_argument('--voltage', metavar='V', nargs="*", type=float,
                        default=[7.4, 11.1, 14.8, 22.2, 22.8, 44.4, 51.8],
                        help='limits the search space to these voltages')

    args = parser.parse_args(args)

    save_all_datapoints(
        iterable=motor_propeller_generator(
            select_propeller=args.propeller,
            select_motor=args.motor),
        voltages=args.voltage,
        fdm_binary=args.fdm,
        propdata=args.propdata,
        output=args.output,
        rpm_limits=not args.no_rpm_limits,
        at20_limits=not args.no_at20_limits)


if __name__ == '__main__':
    run()
