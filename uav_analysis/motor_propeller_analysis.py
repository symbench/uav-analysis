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

import math
import os
import sys


from .components import motor_propeller_generator


def generate_input(bemp_comb, propdata):
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
                        default=-1,
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
    parser.add_argument('--reversible',
                        action='store_true',
                        help='consider only reversible propellers')

    args = parser.parse_args(args)

    generator = motor_propeller_generator(reversible=args.reversible)
    output_header = "Motor,Propeller,Motor_Weight,Propeller_Weight," \
                    "MP_omega_rad,MP_omega_RPM,MP_Voltage,MP_Thrust,MP_Torque,MP_Power,MP_Current,MP_Efficiency," \
                    "MC_omega_rad,MC_omega_RPM,MC_Voltage,MC_Thrust,MC_Torque,MC_Power,MC_Current,MC_Efficiency," \
                    "MaxPower,MaxCur," \
                    "Total_Weight,Total_Thrust,Total_Power,Total_Current\n"

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

        with open('fdm_input.txt', 'w') as file_object:
            file_object.write(generate_input(combination, args.propdata))

        combo_name = "fdm_output.txt"
        cmd = "{} < fdm_input.txt > {}".format(args.fdm, combo_name)
        status = os.system(cmd)
        if status == 2:
            raise KeyboardInterrupt

        MP = None
        MC = None
        count = 0
        with open(combo_name, 'r') as file_object:
            for line in file_object.readlines():
                line = line.strip()

                # if line.startswith("Motor #"):
                #     print(f"\t{line}")
                # if line.startswith("(rad/s)"):
                #     print(f"\t\t  {line}")
                if line.startswith("Max Power 1"):
                    MP = line.split()
                    # print(f"MP {line}")
                elif line.startswith("Max Amps  1"):
                    MC = line.split()
                    # print(f"MC {line}")
            # instrument code to view output
            # count += 1
            # if count == 1:
            #     break

        try:
            for i in range(3, 11):
                float(MP[i])
                float(MC[i])
                break
        except:
            print('\nInvalid fdm output detected for',
                  combination["Motor.Name"], combination["Propeller.Name"])
            continue
        try:
            row = ""
            row += combination["Motor.Name"] + ","
            row += combination["Propeller.Name"] + ","
            row += str(float(combination['Motor.Weight [grams]']) / 1000) + ","
            row += str(float(combination['Propeller.Weight_g']) / 1000) + ","
            row += MP[3] + "," + MP[4] + "," + MP[5] + "," + MP[6] + "," + \
                MP[7] + "," + MP[8] + "," + MP[9] + "," + MP[10] + ","
            row += MC[3] + "," + MC[4] + "," + MC[5] + "," + MC[6] + "," + \
                MC[7] + "," + MC[8] + "," + MC[9] + "," + MC[10] + ","
            row += MC[11] + "," + MC[12] + ","

            total_weight = (float(combination['Motor.Weight [grams]']) / 1000) + (
                float(combination['Propeller.Weight_g']) / 1000)

            # maximum achievable thrust
            total_thrust = None
            total_power = 1e99
            total_current = None
            for M in [MP, MC]:
                t = float(M[6])  # thrust
                p = float(M[8])  # power
                c = float(M[9])  # current
                if math.isnan(t) or math.isnan(c):
                    continue
                if p < total_power:
                    total_power = p
                    total_thrust = t
                    total_current = c
            if total_current is None:
                continue

            row += str(total_weight) + "," + str(total_thrust) + "," + \
                str(total_power) + "," + str(total_current) + "\n"
            res_file.write(row)

            sys.stdout.write('\r')
            sys.stdout.write("{:10.2f}%".format(
                100 * cnt / num_of_combinations))
            sys.stdout.flush()
            cnt += 1
            if args.limit >= 0 and cnt >= args.limit + 1:
                break
        except:
            print(MC)

    print()
    print("---------------------------------------")
    print("Results are saved in file: ", res_fname)
    res_file.close()


if __name__ == '__main__':
    run()
