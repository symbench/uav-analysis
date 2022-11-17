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
        propdata: str,
        speed: float = 20.0) -> str:
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
   control%requested_lateral_speed = {speed}
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
        speed=speed,
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
            elif line.startswith(" Flying    1"):
                linetype = "Flying."
            # elif line.startswith(" Max Power 1"):
            #     linetype = "MaxPower."
            # elif line.startswith(" Max Amps  1"):
            #     linetype = "MaxAmps."
            elif line.startswith("  Requested lateral speed (m/s) ="):
                result["requested_lateral_speed"] = parse(line[33:])
                continue
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
        float(propeller['WEIGHT'])                                      # kg
    result['propeller_diameter'] = float(propeller["DIAMETER"]) * 1e-3  # m
    result['propeller_rpm_max'] = float(propeller["RPM_MAX"])           # rpm
    result['propeller_rpm_min'] = float(propeller["RPM_MIN"])           # rpm
    result['propeller_j_max'] = float(propeller["J_MAX"])               # 1

    result["motor_max_current"] = output_data["MaxVolt.MaxCurrent"] / 0.95  # A
    result["motor_max_power"] = output_data["MaxVolt.MaxPower"] / 0.95      # W

    result["voltage"] = output_data["MaxVolt.Voltage"]           # V

    result["hover_omega_rpm"] = output_data["MaxVolt.OmegaRpm"]  # rpm
    result["hover_thrust"] = output_data["MaxVolt.Thrust"]       # N
    result["hover_power"] = output_data["MaxVolt.Power"]         # W
    result["hover_current"] = output_data["MaxVolt.Current"]     # A
    result["hover_torque"] = output_data["MaxVolt.Torque"]       # Nm

    result["flying_speed"] = output_data.get(
        "requested_lateral_speed", math.nan)                     # m/s
    result["flying_omega_rpm"] = output_data.get("Flying.OmegaRpm", math.nan)
    result["flying_thrust"] = output_data.get("Flying.Thrust", math.nan)
    result["flying_power"] = output_data.get("Flying.Power", math.nan)
    result["flying_current"] = output_data.get("Flying.Current", math.nan)
    result["flying_torque"] = output_data["Flying.Torque"]       # Nm
    result["flying_j"] = result["flying_speed"] / \
        (result["flying_omega_rpm"] / 60.0 * result['propeller_diameter'])

    result["hover_thrust_per_weight"] = result["hover_thrust"] / result["weight"]
    result["hover_power_per_weight"] = result["hover_power"] / result["weight"]
    result["hover_thrust_per_power"] = result["hover_thrust"] / \
        result["hover_power"]

    result["flying_thrust_per_weight"] = result["flying_thrust"] / result["weight"]
    result["flying_power_per_weight"] = result["flying_power"] / result["weight"]
    result["flying_thrust_per_power"] = result["flying_thrust"] / \
        result["flying_power"] if result["flying_power"] > 0 else math.nan

    return result


def generate_multi_datapoints(
        voltages: List[float],
        speed: float,
        fdm_binary: str,
        propdata: str,
        rpm_limits: bool,
        flying_limits: bool,
        motor: Dict[str, Any],
        propeller: Dict[str, Any]) -> List[Dict[str, Any]]:
    result = []
    for voltage in voltages:
        fdm_input = generate_fdm_input(
            motor=motor,
            propeller=propeller,
            voltage=voltage,
            propdata=propdata,
            speed=speed)
        fdm_output = run_new_fdm(
            fdm_binary=fdm_binary,
            fdm_input=fdm_input)
        output_data = parse_fdm_output(
            fdm_output=fdm_output)

        if output_data is None:
            continue

        datapoint = create_single_datapoint(motor, propeller, output_data)

        if datapoint["hover_power"] > datapoint["motor_max_power"]:
            break

        if datapoint["hover_current"] > datapoint["motor_max_current"]:
            break

        if datapoint["hover_thrust"] <= 0 or datapoint["hover_power"] <= 0:
            continue

        if rpm_limits:
            if datapoint["hover_omega_rpm"] > datapoint["propeller_rpm_max"]:
                break

            if datapoint["hover_omega_rpm"] < datapoint["propeller_rpm_min"]:
                continue

        if flying_limits:
            if datapoint["flying_power"] > datapoint["motor_max_power"]:
                break

            if datapoint["flying_current"] > datapoint["motor_max_current"]:
                break

            if datapoint["flying_thrust"] <= 0 or datapoint["flying_power"] <= 0:
                continue

            if rpm_limits:
                if datapoint["flying_omega_rpm"] > datapoint["propeller_rpm_max"]:
                    break

                if datapoint["flying_j"] > datapoint["propeller_j_max"]:
                    break

                if datapoint["flying_thrust"] <= 0:
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
        speed: float,
        fdm_binary: str,
        propdata: str,
        output: str,
        rpm_limits: bool,
        flying_limits: bool):
    voltages = sorted(voltages)

    def task(motor_prop):
        # return [{"motor": motor_prop[0], "propeller": motor_prop[1]}]
        return generate_multi_datapoints(
            voltages=voltages,
            speed=speed,
            fdm_binary=fdm_binary,
            propdata=propdata,
            rpm_limits=rpm_limits,
            flying_limits=flying_limits,
            motor=motor_prop[0],
            propeller=motor_prop[1])

    with open(output, 'w', newline='') as file:
        writer = None
        count = 0

        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
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
    parser.add_argument('--speed', metavar='M/S', default=20.0,
                        type=float, help='target flying speed')

    args = parser.parse_args(args)

    if args.motor not in MOTORS:
        raise ValueError("invalid motor name")
    motor = MOTORS[args.motor]

    if args.propeller not in PROPELLERS:
        raise ValueError("invalid propeller name")
    propeller = PROPELLERS[args.propeller]

    fdm_input = generate_fdm_input(
        motor, propeller, args.voltage, args.propdata, args.speed)
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
                                max_voltage: float,
                                speed: float,
                                propdata: str,
                                fdm: str,
                                ) -> Dict[str, Any]:
    motor = MOTORS[motor]
    propeller = PROPELLERS[propeller]

    motor_rw = float(motor["INTERNAL_RESISTANCE"]) * 0.001  # Ohm
    motor_i_idle = float(motor["IO_IDLE_CURRENT_10V"])      # A
    min_voltage = motor_rw * motor_i_idle + 0.01  # guard value
    med_voltage = (min_voltage + max_voltage) * 0.5

    fdm_input = generate_fdm_input(
        motor, propeller, min_voltage, propdata, speed)
    fdm_output = run_new_fdm(
        fdm_binary=fdm,
        fdm_input=fdm_input)
    output_data = parse_fdm_output(fdm_output)
    min_datapoint = create_single_datapoint(motor, propeller, output_data)

    fdm_input = generate_fdm_input(
        motor, propeller, med_voltage, propdata, speed)
    fdm_output = run_new_fdm(
        fdm_binary=fdm,
        fdm_input=fdm_input)
    output_data = parse_fdm_output(fdm_output)
    med_datapoint = create_single_datapoint(motor, propeller, output_data)

    fdm_input = generate_fdm_input(
        motor, propeller, max_voltage, propdata, speed)
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
        "flying_speed": min_datapoint["flying_speed"],
        "motor_max_current": min_datapoint["motor_max_current"],
        "motor_max_power": min_datapoint["motor_max_power"],
        "min_voltage": min_voltage,
        "min_hover_omega_rpm": min_datapoint["hover_omega_rpm"],
        "min_hover_thrust": min_datapoint["hover_thrust"],
        "min_hover_power": min_datapoint["hover_power"],
        "min_hover_current": min_datapoint["hover_current"],
        "min_flying_omega_rpm": min_datapoint["flying_omega_rpm"],
        "min_flying_thrust": min_datapoint["flying_thrust"],
        "min_flying_power": min_datapoint["flying_power"],
        "min_flying_current": min_datapoint["flying_current"],
        "med_voltage": med_voltage,
        "med_hover_omega_rpm": med_datapoint["hover_omega_rpm"],
        "med_hover_thrust": med_datapoint["hover_thrust"],
        "med_hover_power": med_datapoint["hover_power"],
        "med_hover_current": med_datapoint["hover_current"],
        "med_flying_omega_rpm": med_datapoint["flying_omega_rpm"],
        "med_flying_thrust": med_datapoint["flying_thrust"],
        "med_flying_power": med_datapoint["flying_power"],
        "med_flying_current": med_datapoint["flying_current"],
        "max_voltage": max_voltage,
        "max_hover_omega_rpm": max_datapoint["hover_omega_rpm"],
        "max_hover_thrust": max_datapoint["hover_thrust"],
        "max_hover_power": max_datapoint["hover_power"],
        "max_hover_current": max_datapoint["hover_current"],
        "max_flying_omega_rpm": max_datapoint["flying_omega_rpm"],
        "max_flying_thrust": max_datapoint["flying_thrust"],
        "max_flying_power": max_datapoint["flying_power"],
        "max_flying_current": max_datapoint["flying_current"],
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
    parser.add_argument('voltage', metavar='VOLTAGE',
                        type=float, help='voltage level')
    parser.add_argument('--speed', metavar='M/S', default=20.0,
                        type=float, help='target flying speed')

    args = parser.parse_args(args)

    if args.motor not in MOTORS:
        raise ValueError("invalid motor name")

    if args.propeller not in PROPELLERS:
        raise ValueError("invalid propeller name")

    data = approximate_motor_propeller(
        args.motor, args.propeller, args.voltage, args.speed, args.propdata, args.fdm)
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
    parser.add_argument('--no-flying-limits', action='store_true',
                        help="do not check limts at flying speed")
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
    parser.add_argument('--speed', metavar='M/S', default=20.0,
                        type=float, help='target flying speed')

    args = parser.parse_args(args)

    save_all_datapoints(
        iterable=motor_propeller_generator(
            select_propeller=args.propeller,
            select_motor=args.motor),
        voltages=args.voltage,
        speed=args.speed,
        fdm_binary=args.fdm,
        propdata=args.propdata,
        output=args.output,
        rpm_limits=not args.no_rpm_limits,
        flying_limits=not args.no_flying_limits)


if __name__ == '__main__':
    run()
