#!/usr/bin/env python3
# Copyright (C) 2021, Gyorgy Kalmar
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

import os
import sys

from uav_analysis.bemp_combinations import battery_motor_propeller_generator


def generate_input(bemp_comb, propdata):
    str_return = "&aircraft_data\n"
    str_return += "   aircraft%cname           = 'UAV'\n"
    str_return += "   aircraft%ctype           = 'SymCPS UAV Design'\n"
    str_return += "   aircraft%num_wings       = 0\n"
    str_return += "   aircraft%mass            = {}\n".format(bemp_comb["Aircraft.Mass"])
    str_return += "   aircraft%num_propellers  = 1\n"
    str_return += "   aircraft%num_batteries   = 1\n"
    str_return += "   aircraft%i_analysis_type = 0\n"
    str_return += "\n"
    str_return += "!   Propeller(1 uses components named Prop_0, Motor_0, ESC_0\n"
    str_return += "   propeller(1)%cname = '{}'\n".format(bemp_comb["Propeller.MODEL"])
    str_return += "   propeller(1)%ctype = 'MR'\n"
    str_return += "   propeller(1)%prop_fname = '{}'\n".format(
        os.path.join(propdata, bemp_comb["Propeller.Performance_File"]))
    str_return += "   propeller(1)%radius = {}\n".format(float(bemp_comb["Propeller.DIAMETER"]) / 2.0)
    str_return += "   propeller(1)%Ir = {}\n".format(10.0)  # unknown value but not used
    str_return += "   propeller(1)%motor_fname = '{}'\n".format(bemp_comb["Motor.MODEL"])  # not used
    str_return += "   propeller(1)%KV = {}\n".format(bemp_comb["Motor.KV"])
    str_return += "   propeller(1)%KT = {}\n".format(bemp_comb["Motor.KT"])
    str_return += "   propeller(1)%I_max = {}\n".format(bemp_comb["Motor.MAX_CURRENT"])
    str_return += "   propeller(1)%I_idle = {}\n".format(bemp_comb["Motor.IO_IDLE_CURRENT@10V"])
    str_return += "   propeller(1)%maxpower = {}\n".format(bemp_comb["Motor.MAX_POWER"])
    str_return += "   propeller(1)%Rw = {}\n".format(float(bemp_comb["Motor.INTERNAL_RESISTANCE"]) / 1000.0)
    str_return += "   propeller(1)%icontrol = 1\n"
    str_return += "   propeller(1)%ibattery = 1\n"
    str_return += "   propeller(1)%spin = 1\n"
    str_return += "\n"
    str_return += "!   Battery(1) is component named: Battery_0\n"
    str_return += "   battery(1)%num_cells = {}\n".format(int(bemp_comb["Battery.NUMBER_OF_CELLS"][0]))
    str_return += "   battery(1)%voltage = {}\n".format(bemp_comb["Battery.VOLTAGE"])
    str_return += "   battery(1)%capacity = {}\n".format(bemp_comb["Battery.CAPACITY"])
    str_return += "   battery(1)%C_Continuous = {}\n".format(bemp_comb["Battery.CONT_DISCHARGE_RATE"])
    str_return += "   battery(1)%C_Peak = {}\n".format(bemp_comb["Battery.PEAK_DISCHARGE_RATE"])
    str_return += "/\n"

    return str_return


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--propdata',
                        default=os.path.relpath(os.path.join(
                            os.path.dirname(__file__), 'data', 'propeller')),
                        type=str,
                        metavar='DIR',
                        help="path to propeller data directory")
    parser.add_argument('--output',
                        default='result.csv',
                        type=str,
                        metavar='FILENAME',
                        help="output file name")
    parser.add_argument('--limit',
                        default=-1,
                        type=int,
                        metavar='NUM',
                        help="process only LIMIT number of combinations")
    parser.add_argument('--fdm',
                        default='new_fdm_step0',
                        type=str,
                        metavar='PATH',
                        help="path to fdm executable")
    parser.add_argument('--propeller',
                        metavar='NAME',
                        help='limits the search space to this propeller')
    parser.add_argument('--motor',
                        metavar='NAME',
                        help='limits the search space to this motor')
    parser.add_argument('--battery',
                        metavar='NAME',
                        help='limits the search space to this battery')
    args = parser.parse_args(args)

    generator = battery_motor_propeller_generator(True)
    output_header = "Battery,Motor,Propeller,Battery_Weight,Battery_Capacity,Motor_Weight,Propeller_Weight," \
                    "MV_omega_rad,MV_omega_RPM,MV_Voltage,MV_Thrust,MV_Torque,MV_Power,MV_Current,MV_Efficiency," \
                    "MP_omega_rad,MP_omega_RPM,MP_Voltage,MP_Thrust,MP_Torque,MP_Power,MP_Current,MP_Efficiency," \
                    "MC_omega_rad,MC_omega_RPM,MC_Voltage,MC_Thrust,MC_Torque,MC_Power,MC_Current,MC_Efficiency," \
                    "MaxPower,MaxCur,PeakCur,ContCur," \
                    "Aircraft_Weight,Aircraft_Thrust,Aircraft_Thrust2Weight,Aircraft_FlightTime\n"
    res_fname = args.output
    res_file = open(res_fname, 'w')
    res_file.write(output_header)
    cnt = 1
    if args.limit == -1:
        num_of_combinations = 200_000  # donno yet
    else:
        num_of_combinations = args.limit
    print("Progress: ")
    for combination in generator:
        if args.propeller and args.propeller != combination["Propeller.Name"]:
            continue
        if args.motor and args.motor != combination["Motor.Name"]:
            continue
        if args.battery and args.battery != combination["Battery.Name"]:
            continue

        combination['Aircraft.Mass'] = 0.347232355870562 + float(combination['Battery.WEIGHT']) + \
            4 * float(combination['Motor.WEIGHT']) + 4 * float(combination['Propeller.Weight'])

        with open('fdm_input.txt', 'w') as file_object:
            file_object.write(generate_input(combination, args.propdata))

        combo_name = "fdm_output.txt"
        cmd = "{} < fdm_input.txt > {}".format(args.fdm, combo_name)
        os.system(cmd)

        MV = None
        MP = None
        MC = None
        with open(combo_name, 'r') as file_object:
            for line in file_object.readlines():
                line = line.strip()
                if line.startswith("Max Volt  1"):
                    MV = line.split()
                elif line.startswith("Max Power 1"):
                    MP = line.split()
                elif line.startswith("Max Amps  1"):
                    MC = line.split()

        try:
            for i in range(3, 11):
                float(MV[i])
                float(MP[i])
                float(MC[i])
        except:
            print('\nInvalid fdm output detected for', combination["Battery.Name"],
                  combination["Motor.Name"], combination["Propeller.Name"])
            continue

        row = ""
        row += combination["Battery.Name"] + ","
        row += combination["Motor.Name"] + ","
        row += combination["Propeller.Name"] + ","
        row += combination['Battery.WEIGHT'] + ","
        row += combination['Battery.CAPACITY'] + ","
        row += combination['Motor.WEIGHT'] + ","
        row += combination['Propeller.Weight'] + ","
        row += MV[3] + "," + MV[4] + "," + MV[5] + "," + MV[6] + "," + \
            MV[7] + "," + MV[8] + "," + MV[9] + "," + MV[10] + ","
        row += MP[3] + "," + MP[4] + "," + MP[5] + "," + MP[6] + "," + \
            MP[7] + "," + MP[8] + "," + MP[9] + "," + MP[10] + ","
        row += MC[3] + "," + MC[4] + "," + MC[5] + "," + MC[6] + "," + \
            MC[7] + "," + MC[8] + "," + MC[9] + "," + MC[10] + ","
        row += MC[11] + "," + MC[12] + "," + MC[13] + "," + MC[14] + ","

        # estimations for full thrust flight
        aircraft_thrust = 4 * float(MV[6])
        aircraft_thrust2weight = aircraft_thrust / combination['Aircraft.Mass']
        aircraft_flight_time = float(combination['Battery.CAPACITY']) * 0.8 / (4 * float(MV[9]))

        row += str(combination['Aircraft.Mass']) + "," + str(aircraft_thrust) + "," + \
            str(aircraft_thrust2weight) + "," + str(aircraft_flight_time) + "\n"
        res_file.write(row)

        sys.stdout.write('\r')
        sys.stdout.write("{:10.2f}%".format(100 * cnt / num_of_combinations))
        sys.stdout.flush()
        cnt += 1
        if args.limit >= 0 and cnt >= args.limit + 1:
            break
    print()
    print("---------------------------------------")
    print("Results are saved in file: ", res_fname)
    res_file.close()


if __name__ == '__main__':
    run()
