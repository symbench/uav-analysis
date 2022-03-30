from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import csv 
import os
import pandas as pd

from sklearn.preprocessing import StandardScaler

"""
Predict the mass properties of a wing given its geometry parameters

inputs: dictionary of {var_name: [values]} to predict the outputs
outputs: dictionary of {var_name: [values]} of target outputs

"""
class WingPredictor():
	def __init__(self, 
	inputs, 
	outputs,
	in_vars= ["chord", "span", "max_load", "thickness"],
	out_vars= ["coordIxx", "coordIxz", "coordIyy", "coordIzx", "coordIzz", "cgIxx", "cgIyy", "cgIzz"]):
		assert isinstance(inputs, dict)
		assert isinstance(outputs, dict)
		self.inputs = inputs
		self.outputs = outputs
		self.in_vars = in_vars
		self.out_vars = out_vars

	def select_features(self, var_idx):
		self.in_features = np.vstack([self.inputs[var] for var in self.in_vars]).T  # (152, 4)
		self.out_features = np.vstack([self.outputs[self.out_vars[var_idx]]]).T  # (152, 1)
		print(f"in_features shape: {self.in_features.shape}")
		print(f"out_features shape: {self.out_features.shape}")

	def scale_features(self):
		scaler = StandardScaler()
		self.in_scaled = scaler.fit_transform(self.in_features)
		self.out_scaled = scaler.fit_transform(self.out_features)

	def fit(self):
		self.regs = {ov: None for ov in self.out_vars}
		print(self.regs)
		for i in range(len(self.out_vars)):
			print(f"fitting input: {self.in_vars} to {self.out_vars[i]}")
			self.select_features(i)
			self.scale_features()
			in_train, in_test, out_train, out_test = train_test_split(self.in_scaled, self.out_scaled, test_size=0.10, random_state=42)
			reg = LinearRegression().fit(in_train, out_train)
			cod = reg.score(in_test, out_test)
			print(f"test score: {cod}")
			print(f"coefficients: {reg.coef_}")
			print(f"intercept: {reg.intercept_}")
			self.regs[self.out_vars[i]] = (reg, reg.score(in_test, out_test))
	
	def predict(self, inputs, out_var_name):
		assert isinstance(inputs, np.ndarray)
		assert inputs.shape[1] == len(self.in_vars)
		assert out_var_name in self.out_vars
		return self.regs[out_var_name].predict(inputs)

	def viz(self):
		x_labels = self.out_vars
		vals = sorted([self.regs[ov][1] for ov in self.out_vars])
		plt.xlabel(x_labels)
		plt.ylabel("R^2 value")
		plt.plot(vals)
		plt.title("LinReg - mass properties and determination coefficients")
		plt.show()

def run(data_path = os.path.join(os.path.abspath(os.getcwd()), "uav_analysis", "data_hackathon2_uam")):

	# input data 
	wing_analysis_files = [f for f in os.listdir(data_path) if "wing_analysis_pareto" in f]
	#  output data
	massprops_data_path = os.path.join(data_path, "wing_massprops")
	wing_massprops_files = [f for f in os.listdir(massprops_data_path) if "_pareto.csv" in f]

	wingprops = ["chord", "span", "max_load", "thickness"]
	input_data = {wp: [] for wp in wingprops}

	for data_file in wing_analysis_files:
		df = pd.read_csv(os.path.join(data_path, data_file))
		for wp in input_data.keys():
			if wp != "thickness":
				input_data[wp].extend(df[wp].values)
		
		pct_thickness = np.array([.01 * int(str(x.split(" ")[1][-2:])) for x in df['profile'].values])
		input_data["thickness"].extend(df['chord'].values * pct_thickness)

	# convert lists to numpy arrays
	for prop, data in input_data.items():
		input_data[prop] = np.array(data)

	# output data
	massprops = ["coordIxx", "coordIxz", "coordIyy", "coordIzx", "coordIzz", "cgIxx", "cgIyy", "cgIzz"]

	output_data = {mp: [] for mp in massprops}	

	for data_file in wing_massprops_files:
		df = pd.read_csv(os.path.join(massprops_data_path, data_file))
		#print(df)

		for mp in output_data.keys():
			output_data[mp].extend(df[mp].values)

	for prop, data in output_data.items():
		output_data[prop] = np.array(data)


	# predict mass properties from wing geometry parameters
	wp = WingPredictor(input_data, output_data)
	wp.fit()
	wp.viz()


if __name__ == "__main__":
	#  path hack for direct calls to script during development
	data_path = os.path.join(os.getcwd(), "data_hackathon2_uam")
	run(data_path)