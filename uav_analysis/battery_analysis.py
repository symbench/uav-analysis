import math
import sys
from typing import List, Iterable, Generator

import sympy
from uav_analysis.bemp_combinations_hackathon2_uam import BATTERIES


def combination_generator(
        batteries: Iterable[str],
        max_series: int,
        max_parallel: int
):
    for b in batteries:
        for s in range(1, max_series + 1):
            for p in range(1, max_parallel + 1):
                yield {
                    "battery": b,
                    "series_count": s,
                    "parallel_count": p,
                }


def calc_volume(battery_volume: float,
                series_count: int,
                parallel_count: int) -> float:
    return parallel_count * (series_count * battery_volume)


def calc_weight(battery_weight: float,
                series_count: int,
                parallel_count: int) -> float:
    return parallel_count * (series_count * battery_weight)


def calc_power(battery_cap: float,
               battery_voltage: float,
               series_count: int,
               parallel_count: int) -> float:
    return parallel_count * battery_cap * series_count * battery_voltage


def generate_csv(fname, constant_capacity):
    try:
        output_file = open(fname, 'w')
    except OSError:
        print("Could not open file:", fname)
        sys.exit()
    output_file.write("NAME,SERIES_COUNT,PAR_COUNT,VOLUME,WEIGHT,POWER\n")

    comb_generator = combination_generator(BATTERIES, 10, 10)
    for c in comb_generator:
        print("Processing combination: ", c)
        battery_volume = float(BATTERIES[c["battery"]]["Volume [mm^3]"])
        battery_weight = float(BATTERIES[c["battery"]]["Weight [kg]"])
        if constant_capacity == -1.0:
            battery_capacity = float(BATTERIES[c["battery"]]["Capacity [Ah]"])
        else:
            battery_capacity = constant_capacity
        battery_voltage = float(BATTERIES[c["battery"]]["Min Voltage [V]"])
        row_string = "{},{},{},{},{},{}\n".format(c["battery"],
                                                  c["series_count"],
                                                  c["parallel_count"],
                                                  calc_volume(battery_volume,
                                                              c["series_count"], c["parallel_count"]),
                                                  calc_weight(battery_weight,
                                                              c["series_count"], c["parallel_count"]),
                                                  calc_power(battery_capacity,
                                                             battery_voltage,
                                                             c["series_count"], c["parallel_count"]))
        output_file.write(row_string)
        # generate_rows(output_file, b, constant_capacity)
    output_file.close()
    print("Result table is saved to:", fname)


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
    action.add_argument('-g', '--g_csv', nargs='?', metavar="PATH",
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
        print("Analyzed battery: ", args.a_batt)
        battery_analyzer(args.a_batt)


if __name__ == '__main__':
    run()
