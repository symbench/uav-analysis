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
from dataclasses import dataclass
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sympy
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVR

from uav_analysis.fdm_review import FdmReport
from uav_analysis import func_approx as fa
from uav_analysis.mass_properties_hackathon1 import (inertia_matrix, translate_inertia)


class MassPropertyDataset():
	def __init__(self, component_name: str, data_path: str, input_names: list, output_names: list, target: str):

		self.component_name = component_name
		self.data_path = data_path
		self.input_names = input_names

		self.output_names = output_names
		assert target in self.output_names
		self.target = target
		self.output_names = self.output_names
		self.load_data()
		#self.scale_features()

	
	def load_data(self):
		# read all csvs into a single dataframe to initialize the MassPropertyDataset
		massprops_files = os.listdir(self.data_path)
		self.df = pd.concat([pd.read_csv(os.path.join(self.data_path, f)) for f in massprops_files])
		stem = "Component_param_"
		self.invar_keys = [stem + iv for iv in self.input_names]

		self.input_df = self.df[[ik for ik in self.invar_keys]]
		self.input_dict = {k: self.input_df[[k]].values for k in self.invar_keys}
		
		self.inputs_data = self.input_df.values

		self.output_df = self.df[[ok for ok in self.output_names]]
		self.output_dict = self.output_df.to_dict()
		self.target_data = self.output_df[[self.target]].values
		

		print(f"type input data: {type(self.inputs_data)}, shape: {self.inputs_data.shape}")
		print(f"type target data: {type(self.target_data)}, shape: {self.target_data.shape}")

	
	def update_target(self, new_target: str):
		assert new_target in self.output_names
		print(f"changing target {self.target} --> {new_target}")
		self.target = new_target
		self.target_data = self.output_df[[self.target]].values
		# print(f"updated target to {self.target} to {self.target_data[:5]}")


	def scale_features(self):
		scaler = StandardScaler() #  center to zero mean and unit variance
		self.in_scaled = scaler.fit_transform(self.inputs)
		self.out_scaled = scaler.fit_transform(self.outputs)
		print(f"{self.component_name} scaled [{type(self.in_scaled)}, {self.in_scaled.shape}]")
		print(f"{self.component_name} scaled [{type(self.out_scaled)}, {self.out_scaled.shape}]")

	def output_head(self):
		print(self.output_df.head())


	def viz(self, input_trio: list, output_trio: list):
		# plot specified features in 3D (side by side subplots from inputs to outputs)
		pass


"""

Predict the mass properties of a UAM component given its geometry parameters from Creo

"""
class MassPropertyPredictor():  # updated version will take MassPropertyDataset as input
	def __init__(self, name: str, dataset: MassPropertyDataset):
		self.name = name
		self.dataset = dataset
		self.predictors = {}

	def fit(self):
		#  todo maybe modify this expression for different targets
		#  todo maybe visualize results
		#  ax^3 + bx^2 + cx + d
		a = sympy.Symbol('a')
		b = sympy.Symbol('b')
		c = sympy.Symbol('c')
		d = sympy.Symbol('d')
		x = sympy.Symbol('x')
		f = a * (x ** 3) + b * (x ** 2) + c * x + d

		print(f"target data {self.dataset.target_data} shape {self.dataset.target_data.shape}")
		for ov in self.dataset.output_names:
			self.dataset.update_target(ov)
			subs, error = fa.approximate(f, self.dataset.input_dict, self.dataset.target_data.ravel())
			self.predictors[ov] = (subs, error)


	def viz(self):
		pass



def run(data_path: str = os.path.join(os.path.abspath(os.getcwd()), "uav_analysis", "data_hackathon2_uam")):

	wing_massprops_data_path = os.path.join(data_path, "wing_massprops")
	wing_invar_names = ["CHORD_1", "SPAN", "THICKNESS"]
	wing_outvar_names = ["coordIxx","coordIxz","coordIyy","coordIzx","coordIzz", "cgIxx", "cgIyy", "cgIzz"]
	wing_dataset = MassPropertyDataset("wing", wing_massprops_data_path, wing_invar_names, wing_outvar_names, "coordIxx")
	print(" ---- wing dataset ---- ")
	for o in wing_outvar_names[1:]:
		wing_dataset.update_target(o)
	wing_dataset.output_head()
	print(" ---------------------- ")

	cylinder_massprops_data_path = os.path.join(data_path, "cylinder_massprops")
	cylinder_invar_names = ["DIAMETER", "LENGTH", "PORT_THICKNESS"]
	cylinder_outvar_names = ["coordIxx","coordIyy", "coordIzz", "cgIxx", "cgIyy", "cgIzz"]
	cylinder_dataset = MassPropertyDataset("cylinder", cylinder_massprops_data_path, cylinder_invar_names, cylinder_outvar_names, "coordIxx")
	print(" ---- cylinder dataset ---- ")
	for o in cylinder_outvar_names[1:]:
		cylinder_dataset.update_target(o)
	cylinder_dataset.output_head()
	print(" -------------------------- ")

	wp = MassPropertyPredictor("wing", wing_dataset)
	wp.fit()
	cp = MassPropertyPredictor("cylinder", cylinder_dataset)
	cp.fit()
if __name__ == "__main__":
	#  path hack for direct calls to script during development
	data_path = os.path.join(os.getcwd(), "data_hackathon2_uam")
	
	run(data_path)
