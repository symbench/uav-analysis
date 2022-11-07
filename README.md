# UAV analysis playground

## Installation
* Clone it with `git clone git@github.com:symbench/uav-analysis.git --recurse-submodules`
* Install it with `pip3 install -e .`
* Update it with `git pull --recurse-submodules`
* You also need to install the `constraint-prog` repo.

## Hackathon 2 (2022. November 7)
* Used `athens_graphops query --property-table-csv Battery > BatteryAll.csv` to ontain the
  propeties of all batteries, and did the same for motors and propellers. Manually opened these
  and kept those marked with UAV or Both and saved them into the `data_hackathon1` direcotry.
  Used the `./uav_analysis/components.py` script to extend the `Propeller.csv` table with
  maximum and minimum RPM values.

