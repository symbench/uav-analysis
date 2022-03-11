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

import math
import os
import sys


from uav_analysis.bemp_combinations_hackathon2_uam import battery_motor_propeller_generator


def generate_input(bemp_comb, propdata, fixedBatteryParams=False):
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
    str_return += "   propeller(1)%cname = '{}'\n".format(bemp_comb["Propeller.Name"])
    str_return += "   propeller(1)%ctype = 'MR'\n"
    str_return += "   propeller(1)%prop_fname = '{}'\n".format(
        os.path.join(propdata, bemp_comb["Propeller.Performance_File"]))
    str_return += "   propeller(1)%radius = {}\n".format(float(bemp_comb["Propeller.Diameter_mm"]) / 2.0)
    str_return += "   propeller(1)%Ir = {}\n".format(10.0)  # unknown value but not used
    str_return += "   propeller(1)%motor_fname = '{}'\n".format(bemp_comb["Motor.Name"])  # not used
    str_return += "   propeller(1)%KV = {}\n".format(bemp_comb["Motor.KV [RPM/V]"])
    str_return += "   propeller(1)%KT = {}\n".format(bemp_comb["Motor.KT [Nm/A]"])
    str_return += "   propeller(1)%I_max = {}\n".format(bemp_comb["Motor.Max Current [A]"])
    str_return += "   propeller(1)%I_idle = {}\n".format(bemp_comb["Motor.Io Idle Current@10V [A]"])
    str_return += "   propeller(1)%maxpower = {}\n".format(bemp_comb["Motor.Max Power [W]"])
    str_return += "   propeller(1)%Rw = {}\n".format(float(bemp_comb["Motor.Internal Resistance [mOhm]"]) / 1000.0)
    str_return += "   propeller(1)%icontrol = 1\n"
    str_return += "   propeller(1)%ibattery = 1\n"
    str_return += "   propeller(1)%spin = 1\n"
    str_return += "\n"
    str_return += "!   Battery(1) is component named: Battery_0\n"
    str_return += "   battery(1)%num_cells = {}\n".format(int(bemp_comb["Battery.Number of Series Cells"][0]))
    

    if fixedBatteryParams:
        bemp_comb["Battery.Min Voltage [V]"] = "1000"  # hardcode 1000 V -- doesn't work
        bemp_comb["Battery.Capacity [Ah]"] = "100" # harcode 100 Ah
        bemp_comb["Battery.Cont. Discharge Rate [C]"] = "25"  # hardcode 25 A
        bemp_comb["Battery.Peak Discharge Rate [C]"] = "50"  # hardcode 50 A

    str_return += "   battery(1)%voltage = {}\n".format(
        bemp_comb["Battery.Max Voltage [V]"])

    str_return += "   battery(1)%capacity = {}\n".format(
        bemp_comb["Battery.Capacity [Ah]"])

    
    str_return += "   battery(1)%C_Continuous = {}\n".format(
        bemp_comb["Battery.Cont. Discharge Rate [C]"])

    
    str_return += "   battery(1)%C_Peak = {}\n".format(
        bemp_comb["Battery.Peak Discharge Rate [C]"])
    str_return += "/\n"

    print(f"--- BATTERY PARAMS (fixbatt={fixedBatteryParams}): ---")
    print(f"VOLTAGE: {bemp_comb['Battery.Min Voltage [V]']}")
    print(f"CAPACITY: {bemp_comb['Battery.Capacity [Ah]']}")
    print(f"CONT_DIS: {bemp_comb['Battery.Cont. Discharge Rate [C]']}")
    print(f"PEAK_DIS: {bemp_comb['Battery.Peak Discharge Rate [C]']}")


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
                        default='../flight-dynamics-model/bin/new_fdm',
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
    parser.add_argument('--fixbatt',
                        action='store_true',
                        help='true or false to fix battery voltage, capacity, and discharge rates')
    parser.add_argument('--feature', action='store_true')
    #parser.set_defaults(fixbatt=False)
    
    args = parser.parse_args(args)

    generator = battery_motor_propeller_generator(reversible=False)
    output_header = "Battery,Motor,Propeller,Battery_Weight,Battery_Capacity,Motor_Weight,Propeller_Weight," \
                    "MV_omega_rad,MV_omega_RPM,MV_Voltage,MV_Thrust,MV_Torque,MV_Power,MV_Current,MV_Efficiency," \
                    "MP_omega_rad,MP_omega_RPM,MP_Voltage,MP_Thrust,MP_Torque,MP_Power,MP_Current,MP_Efficiency," \
                    "MC_omega_rad,MC_omega_RPM,MC_Voltage,MC_Thrust,MC_Torque,MC_Power,MC_Current,MC_Efficiency," \
                    "MaxPower,MaxCur,PeakCur,ContCur," \
                    "Aircraft_Weight,Aircraft_Thrust,Aircraft_Thrust2Weight,Aircraft_FlightTime\n"
    
    # modified to consider mainly motor and propeller
    # output_header = "Battery,Motor,Propeller,Battery_Weight,Battery_Capacity,Motor_Weight,Propeller_Weight," \
    #                 "MV_omega_rad,MV_omega_RPM,MV_Voltage,MV_Thrust,MV_Torque,MV_Power,MV_Current,MV_Efficiency," \
    #                 "MP_omega_rad,MP_omega_RPM,MP_Voltage,MP_Thrust,MP_Torque,MP_Power,MP_Current,MP_Efficiency," \
    #                 "MC_omega_rad,MC_omega_RPM,MC_Voltage,MC_Thrust,MC_Torque,MC_Power,MC_Current,MC_Efficiency," \
    #                 "MaxPower,MaxCur,PeakCur,ContCur," \
    #                 "Aircraft_Weight,Aircraft_Thrust,Aircraft_Thrust2Weight,Aircraft_FlightTime\n"
    
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

        # combination['Aircraft.Mass'] = 0.347232355870562 + float(combination['Battery.Weight [kg]']) + \
        #     4 * (float(combination['Motor.Weight [grams]'])/ 1000) + 4 * (float(combination['Propeller.Weight_g']) / 1000)

        # just consider the weight of the motor and propeller, kg 
        combination['Aircraft.Mass'] = (
            float(combination['Motor.Weight [grams]']) / 1000) + (
                float(combination['Propeller.Weight_g']) / 1000) 
                
        with open('fdm_input.txt', 'w') as file_object:
            file_object.write(generate_input(combination, args.propdata, args.fixbatt))

        combo_name = "fdm_output.txt"
        cmd = "{} < fdm_input.txt > {}".format(args.fdm, combo_name)
        status = os.system(cmd)
        if status == 2:
            raise KeyboardInterrupt

        MV = None
        MP = None
        MC = None
        count = 0
        with open(combo_name, 'r') as file_object:
            for line in file_object.readlines():
                line = line.strip()

                if line.startswith("Motor #"):
                    print(f"\t{line}")
                if line.startswith("(rad/s)"):
                    print(f"\t\t  {line}")
                if line.startswith("Max Volt  1"):
                    MV = line.split()
                    print(f"MV {line}")
                elif line.startswith("Max Power 1"):
                    MP = line.split()
                    print(f"MP {line}")
                elif line.startswith("Max Amps  1"):
                    MC = line.split()
                    print(f"MC {line}")
            # instrument code to view output
            count += 1
            if count == 1:
                break

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
        row += combination['Battery.Weight [kg]'] + ","
        row += combination['Battery.Capacity [Ah]'] + ","
        row += str(float(combination['Motor.Weight [grams]']) / 1000) + ","
        row += str(float(combination['Propeller.Weight_g']) / 1000) + ","
        row += MV[3] + "," + MV[4] + "," + MV[5] + "," + MV[6] + "," + \
            MV[7] + "," + MV[8] + "," + MV[9] + "," + MV[10] + ","
        row += MP[3] + "," + MP[4] + "," + MP[5] + "," + MP[6] + "," + \
            MP[7] + "," + MP[8] + "," + MP[9] + "," + MP[10] + ","
        row += MC[3] + "," + MC[4] + "," + MC[5] + "," + MC[6] + "," + \
            MC[7] + "," + MC[8] + "," + MC[9] + "," + MC[10] + ","
        row += MC[11] + "," + MC[12] + "," + MC[13] + "," + MC[14] + ","

        # maximum achievable thrust
        aircraft_thrust = 1e99
        aircraft_current = None
        for M in [MV, MP, MC]:
            t = float(M[6])  # thrust
            c = float(M[9])  # current
            if math.isnan(t) or math.isnan(c):
                continue
            if t < aircraft_thrust:
                aircraft_thrust = t
                aircraft_current = c
        if aircraft_current is None:
            continue

        aircraft_thrust2weight = aircraft_thrust / (combination['Aircraft.Mass'] * 9.81)
        aircraft_flight_time = float(combination['Battery.Capacity [Ah]']) * 0.8 / (4 * aircraft_current)

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
