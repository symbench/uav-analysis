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
