import math
import sys
from typing import List

import sympy
from uav_analysis.bemp_combinations_hackathon2_uam import BATTERIES
from uav_analysis.bemp_combinations_hackathon2_uam import WINGS


def generate_csv(fname, constant_capacity):
    try:
        output_file = open(fname, 'a')
    except OSError:
        print("Could not open file:", fname)
        sys.exit()
    output_file.write("NAME,SERIES_COUNT,PAR_COUNT,VOLUME,WEIGHT,POWER\n")
    for b in BATTERIES:
        print("Processing battery: ", b)
        generate_rows(output_file, b, constant_capacity)
    output_file.close()
    print("Result table is saved to:", fname)


def generate_rows(file_to_append, batt_name, constant_capacity, parallel_max=10):
    batt_params = BATTERIES[batt_name]
    if constant_capacity == -1.0:
        batt_cap = float(batt_params["Capacity [Ah]"])
    else:
        print("Battery capacity is replaced with ", constant_capacity)
        batt_cap = constant_capacity
    batt_voltage = float(batt_params["Min Voltage [V]"])
    batt_volume = float(batt_params["Volume [mm^3]"])
    batt_weight = float(batt_params["Weight [kg]"])
    batt_max_voltage = float(batt_params["Max Voltage [V]"])  # which one to use? 1)
    batt_num_of_series_cells = int(batt_params["Number of Series Cells"])  # which one to use? 2)

    for s in range(1, batt_num_of_series_cells + 1):
        series_voltage = s * batt_voltage
        series_volume = s * batt_volume
        series_power = batt_cap * series_voltage
        series_weight = s * batt_weight
        for p in range(1, parallel_max):
            total_voltage = series_voltage  # not written but useful
            total_power = p * series_power
            total_volume = p * series_volume
            total_weight = p * series_weight

            row_string = "{},{},{},{},{},{}\n".format(batt_name, s, p, total_volume, total_weight, total_power)
            file_to_append.write(row_string)


def battery_analyzer_sympy(batt_name="Tattu 22Ah Li", ):
    capacity = sympy.Symbol("capacity")
    voltage_request = sympy.Symbol("voltage_request")
    base_voltage = sympy.Symbol("base_voltage")
    module_volume = sympy.Symbol("module_volume")
    volume_percent = sympy.Symbol("volume_percent")
    chord_1 = sympy.Symbol("chord_1")
    chord_2 = sympy.Symbol("chord_2")
    thickness = sympy.Symbol("thickness")
    span = sympy.Symbol("span")

    series_count = sympy.ceiling(voltage_request / base_voltage)
    min_pack_volume = series_count * module_volume
    volume_ratio = volume_percent / 100.0
    root_chord = sympy.Max(chord_1, chord_2)
    tip_chord = sympy.Min(chord_1, chord_2)
    A = root_chord / 2
    B = thickness / 100 * A
    C = tip_chord / 2
    D = thickness / 100 * C
    available_volume = 1 / 6 * span * (A * B + C * D + ((A + C) * (B + D)))
    goal_volume = available_volume * volume_ratio
    p_count = sympy.floor(goal_volume / min_pack_volume)
    mega_pack_capacity = capacity * p_count  # output
    actual_voltage = base_voltage * series_count  # output
    print("Symbolic solution with substitution: ", end='')
    print(mega_pack_capacity.evalf(subs={capacity: float(BATTERIES[batt_name]["Capacity [Ah]"]),
                                         span: 3000.0,
                                         volume_percent: 10.0,
                                         thickness: 12.0,
                                         chord_1: 1000.0,
                                         chord_2: 1000.0,
                                         module_volume: float(BATTERIES[batt_name]["Volume [mm^3]"]),
                                         voltage_request: 51.8,
                                         base_voltage: float(BATTERIES[batt_name]["Min Voltage [V]"])}))


def battery_analyzer(batt_name="Tattu 22Ah Li", ):
    capacity = float(BATTERIES[batt_name]["Capacity [Ah]"])
    voltage_request = 51.8  # input
    base_voltage = float(BATTERIES[batt_name]["Min Voltage [V]"])
    module_volume = float(BATTERIES[batt_name]["Volume [mm^3]"])
    volume_percent = 10.0
    chord_1 = 1000.0  # does not match wings csv
    chord_2 = 1000.0  # does not match wings csv
    thickness = 12.0  # where does it come from?
    span = 3000.0  # does not match wings csv

    series_count = math.ceil(voltage_request / base_voltage)
    min_pack_volume = series_count * module_volume
    volume_ratio = volume_percent / 100

    c_continuous = float(BATTERIES[batt_name]
                         ["Cont. Discharge Rate [C]"])  # not used
    c_peak = float(BATTERIES[batt_name]["Peak Discharge Rate [C]"])  # not used

    root_chord = max([chord_1, chord_2])
    tip_chord = min([chord_1, chord_2])
    A = root_chord / 2
    B = thickness / 100 * A
    C = tip_chord / 2
    D = thickness / 100 * C

    slope = (C - A) / span  # not used
    available_volume = 1 / 6 * span * (A * B + C * D + ((A + C) * (B + D)))
    goal_volume = available_volume * volume_ratio

    p_count = math.floor(goal_volume / min_pack_volume)
    mega_pack_capacity = capacity * p_count  # output
    actual_voltage = base_voltage * series_count  # output
    print("Direct solution: ", end='')
    print(mega_pack_capacity)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser()
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('-a', '--a_batt', nargs='?', metavar="STR",
                        const="Tattu 22Ah Li", help="Analyzes given battery")
    action.add_argument('--g_csv', nargs='?', metavar="PATH",
                        const="data_hackathon2_uam/Battery_analysis.csv",
                        help="Generates battery analysis table")
    parser.add_argument('--const_C', nargs=1, metavar="FLOAT",
                        default=-1.0, help="Ignores actual battery capacity and replaces with 'const_C'")

    args = parser.parse_args(args)
    if args.g_csv is not None:
        if isinstance(args.const_C, List):
            const_C = float(args.const_C[0])
        else:
            const_C = float(args.const_C)
        generate_csv(args.g_csv, const_C)
    else:
        print("Used battery: ", args.a_batt)
        battery_analyzer(args.a_batt)
        battery_analyzer_sympy(args.a_batt)


if __name__ == '__main__':
    run()
