import math
import sympy
from uav_analysis.bemp_combinations_hackathon2_uam import BATTERIES
from uav_analysis.bemp_combinations_hackathon2_uam import WINGS


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
    mega_pack_capacity = capacity * p_count   # output
    actual_voltage = base_voltage * series_count   # output
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
    # inputs
    # bidx = findClassIndex(componentOrdering, "Battery", bn)

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
    mega_pack_capacity = capacity * p_count   # output
    actual_voltage = base_voltage * series_count   # output
    print("Direct solution: ", end='')
    print(mega_pack_capacity)


def run(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--batt_name', type=str, metavar='"BATTERY-COMPONENT-NAME"',
                        help='battery component name', default="Tattu 22Ah Li")
    args = parser.parse_args(args)
    print("Used battery: ", args.batt_name)
    battery_analyzer(args.batt_name)
    battery_analyzer_sympy(args.batt_name)


if __name__ == '__main__':
    run()
