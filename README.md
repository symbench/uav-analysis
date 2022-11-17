# UAV analysis playground

## Installation of uav-analysis
* Clone it with `git clone git@github.com:symbench/uav-analysis.git --recurse-submodules`
* Install it with `pip3 install -e .`
* Update it with `git pull --recurse-submodules`
* You also need to install the `constraint-prog` repo.

## Compilation of new-fdm
* Go into the `flight-dynamics-model` submodule
* `autoreconf -if`
* `./configure`
* `make`
* Do not install it, we will execute it from that subdirectory.

## Hackathon 2 steps (2022. November 7)
* Used `athens-graphops query --property-table-csv Battery > BatteryAll.csv` to ontain the
  propeties of all batteries, and did the same for motors and propellers. Manually opened these
  and kept those marked with UAV or Both and saved them into the `data_hackathon2` direcotry.
  Used the `./uav_analysis/components.py` script to extend the `Propeller.csv` table with
  maximum and minimum RPM values.
* Used `uav-analysis motor-propeller-analysis` to generate the `motor_propeller_analysis.csv` file
  containing all performance data for all combinations of motors, propellers and battery voltages.
  We also generate performance data at 20 m/s air (crusing) speed. Used
  `constraint-prog pareto-front --max weight 0.2 --min thrust 14.0 thrust_at20 10.0 --neg weight power --pos thrust --save motor_propeller_analysis_pareto.csv motor_propeller_analysis.csv` 
  to find one possible pareto front, and saved it into the `data_hackathon2` folder.
* Used `uav-analysis battery-analysis --max-parallel 2 --output battery_analysis.csv` to generate
  all combination of batteries, then pruned it using
  `constraint-prog pareto-front --neg total_weight --pos total_voltage total_capacity total_current --save battery_analysis_pareto.csv battery_analysis.csv`, and saved it the
  `data_hackathon2` folder.

### High power design:

Used `constraint-prog pareto-front --max weight 0.4 --neg weight power --pos thrust_at20 --save motor_propeller_analysis_pareto.csv motor_propeller_analysis.csv` to find high thrust motor
propeller combination, then used `uav-analysis motor-propeller-approximate t_motor_AntigravityMN5008KV340 apc_propellers_13x14` to get the tables, then used `uav-analysis napkin-calculator napkin2`, identified the battery by its weight (which is minimized). 

* 4 motors/propellers: t_motor_AntigravityMN5008KV340, apc_propellers_13x14
* 1 battery: Tattu25C23000mAh6S1PHV
* 2 wings: NACA 0012, chord 55 mm, span 390 mm, load 492 N
