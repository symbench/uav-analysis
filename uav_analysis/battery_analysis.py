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


def generate_csv(
        fname: str,
        constant_capacity: bool,
        max_parallel: int,
        max_series: int,
        min_voltage: float,
        max_voltage: float):
    try:
        output_file = open(fname, 'w')
    except OSError:
        print("Could not open file:", fname)
        sys.exit()
    output_file.write(
        "NAME,BATTERY_VOLUME,BATTERY_WEIGHT,BATTERY_CAPACITY,SERIES_COUNT,PAR_COUNT,TOTAL_VOLUME,TOTAL_WEIGHT,TOTAL_POWER,TOTAL_VOLTAGE\n")

    comb_generator = combination_generator(
        BATTERIES,
        max_series=max_series,
        max_parallel=max_parallel)

    count = 0
    for c in comb_generator:
        # print("Processing combination: ", c)
        battery = BATTERIES[c["battery"]]
        battery_volume = float(battery["Volume [mm^3]"])
        battery_weight = float(battery["Weight [kg]"])
        if not constant_capacity:
            battery_capacity = float(battery["Capacity [Ah]"])
        else:
            battery_capacity = 25.0  # check run_fd_calc.py
        battery_voltage = float(battery["Min Voltage [V]"])

        total_voltage = battery_voltage * c["parallel_count"]
        if total_voltage < min_voltage or total_voltage > max_voltage:
            continue

        row_string = "{},{},{},{},{},{},{},{},{},{}\n".format(
            c["battery"],
            battery_volume,
            battery_weight,
            battery_capacity,
            c["series_count"],
            c["parallel_count"],
            calc_volume(battery_volume,
                        c["series_count"], c["parallel_count"]),
            calc_weight(battery_weight,
                        c["series_count"], c["parallel_count"]),
            calc_power(battery_capacity,
                       battery_voltage,
                       c["series_count"], c["parallel_count"]),
            total_voltage
        )
        output_file.write(row_string)
        count += 1

    output_file.close()
    print("Result table with", count, "rows is saved to:", fname)


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
    action.add_argument('-a', '--a-batt', nargs='?', metavar="STR",
                        const="Tattu 22Ah Li", help="Analyzes given battery")
    action.add_argument('-o', '--output', nargs='?', metavar="PATH",
                        const="battery_analysis.csv",
                        help="Generates battery analysis table")
    parser.add_argument('--const-cap', action="store_true", default=False,
                        help="Ignores actual battery capacity and use 25 Ah")
    parser.add_argument('--max-parallel', type=int, default=500, metavar="NUM",
                        help="sets the maximum parallel count")
    parser.add_argument('--max-series', type=int, default=100, metavar="NUM",
                        help="sets the maximum series count")
    parser.add_argument('--min-voltage', type=float, default=100, metavar="NUM",
                        help="sets the minimum total voltage")
    parser.add_argument('--max-voltage', type=float, default=1000, metavar="NUM",
                        help="sets the maximum total voltage")

    args = parser.parse_args(args)
    if args.output is not None:
        generate_csv(
            fname=args.output,
            constant_capacity=args.const_cap,
            max_parallel=args.max_parallel,
            max_series=args.max_series,
            min_voltage=args.min_voltage,
            max_voltage=args.max_voltage,
        )
    else:
        print("Analyzed battery: ", args.a_batt)
        battery_analyzer(args.a_batt)


if __name__ == '__main__':
    run()
