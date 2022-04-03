#!/usr/bin/env python3
# Copyright (C) 2022, Michael Sandborn
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
import json


"""
Helper class to quickly access fields of the flight dynamics data
given a path to the results folder of interest.
"""
class FdmReport():
	def __init__(self, run_folder: str):
		self.folder = run_folder
		self.load_data()
		self.inp_line_counts = {k: len(v) for k, v in self.out_files.items()}
		self.out_line_counts = {k: len(v) for k, v in self.out_files.items()}


	def load_data(self):
		
		self.inp_files = {}
		self.out_files = {}
		for f in os.listdir(self.folder):
			if f.endswith(".inp"):
				with open(os.path.join(self.folder, f), 'r') as inp_file:
					path = f.split(".")[0][-1]
					self.inp_files["path_"+path+"_inp"] = inp_file.readlines()
			elif f.endswith(".out"):
				with open(os.path.join(self.folder,f), 'r') as out_file:
					path = f.split(".")[0][-1]
					self.out_files["path_"+path+"_out"] = out_file.readlines()
		print(self.inp_files.keys())
	
	def get_vehicle_massprops(self):
		massprops = {}
		lines = self.inp_files["path_1_inp"]
		pattern = re.compile(r"aircraft%[I][x-zX-Z][x-zX-Z]")
		pattern2 = re.compile(r"aircraft%[x-zX-Z]_[cf]+")
		for line in lines:
			match = pattern.finditer(line)
			match2 = pattern2.finditer(line)
			for instance in match:
				values = instance.string.split()
				massprops[values[0]] = values[-1]
			for instance in match2:
				values = instance.string.split()
				massprops[values[0]] = values[-1]

		return massprops

	def get_score_info(self):
		# return vehicle params
		return {k: v[v.index(" #Metrics\n"):] for k, v in self.out_files.items()}

	def get_trim_state_count(self, path):
		# return trim state count
		assert path in ["1","3","4","5"]
		lines = self.out_files["path_"+path+"_out"]
		key = "  This routine finds steady solutions for lateral speeds of 0 to 50 m/s (Unorth).\n"
		idxs = [idx for idx in range(len(lines)) if lines[idx] == key]
		print(idxs)
		
if __name__ == "__main__":
	report = FdmReport(run_folder=sys.argv[1])
	print(report.get_vehicle_massprops())
