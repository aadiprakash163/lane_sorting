import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# import cv2
import math
import time
import threading
import random
from mip import *
np.set_printoptions(threshold=np.inf)
import copy 

N_LANES = 3

def sort_vehicles(vehicles, FRAME_LENGTH, LEN, new_frame):
	
	
	print("Positions of all the vehicles in this frame:")
	print(list(map(lambda v: (v.id, v.pos), vehicles)))
	
	SG = 2*float(LEN)/3
	max_vehicle = int(FRAME_LENGTH/(LEN + SG))
	unsorted_veh = []
	sorted_veh = []
	veh_in_lane = [[], [], []]

	for v in vehicles :
		if v.lane == 1 : veh_in_lane[0].append(v)
		elif v.lane == 2 : veh_in_lane[1].append(v)
		else: veh_in_lane[2].append(v)
	

	all_vehicles_sorted = False

	states = []
	just_formed = True
	while not all_vehicles_sorted:
		vehicles = sorted(vehicles, key = lambda x: x.pos, reverse = True)
		positions = []	
		sorted_vehicles_index = []
		unsorted_vehicles_index = []		
		supporting_vehicles_index = []	

		# Get list of sorted and unsorted vehicles
		for i in range(len(vehicles)):		
			if vehicles[i].lane == vehicles[i].destination:
				sorted_vehicles_index.append(i)
			else:
				unsorted_vehicles_index.append(i)

			positions.append(vehicles[i].pos)
		

		solution_found = False	
		N = 0
		lane_counter = [0 for i in range(N_LANES)]
		for vehicle in vehicles:
			l_s = vehicle.lane-1
			l_d = vehicle.destination-1
			if l_s == l_d:
				lane_counter[l_s] += 1
			else:
				if l_s < l_d:
					for i in range(l_s, l_d):
						lane_counter[i] += 1
					lane_counter[l_d] += 1
				else:
					for i in range(l_d, l_s):
						lane_counter[i] += 1
					lane_counter[l_s] += 1
			
		for i in range(N_LANES):
			if lane_counter[i] > max_vehicle:
				N += lane_counter[i] - max_vehicle

		if just_formed and (N >= len(unsorted_vehicles_index) or len(unsorted_vehicles_index) == 0):
			just_formed = False
			# print("Need to merge frames")
			print("Rearranging frame")
			m2 = Model('vehicle_arranger')
			m2.verbose = 0
			delta_pos2 = [m2.add_var(lb = -FRAME_LENGTH, ub = FRAME_LENGTH) for i in range(len(vehicles))]

			for i in range(len(vehicles)):
				# Constraints to keep vehicles in the frame
				m2 += (positions[i] + delta_pos2[i]) >= (LEN + SG/2)
				m2 += (positions[i] + delta_pos2[i]) <= FRAME_LENGTH - SG/2
			
				current_lane = vehicles[i].lane			
				for j in range(i+1, len(vehicles)):
					if(vehicles[j].lane == current_lane):
						if vehicles[i].pos > vehicles[j].pos:
							m2 += (positions[i] + delta_pos2[i]) - (positions[j] + delta_pos2[j]) >= (LEN + SG)
						else:
							m2 += (positions[j] + delta_pos2[j]) - (positions[i] + delta_pos2[i]) >= (LEN + SG)

			Obj = m2.add_var()
			m2 += xsum(delta_pos2[i] for i in range(len(vehicles))) <= Obj

			m2.objective = minimize(Obj)

			m2.optimize()

			if m2.num_solutions:
				# print("Rearranging vehicles in the frame")
				for i in range(len(delta_pos2)):
				
					# print("VEHICLE POS: %r" %vehicles[i].pos)
					vehicles[i].pos = vehicles[i].pos + delta_pos2[i].x
					
				states.append(copy.deepcopy(vehicles))
				if N > 0 and N >= len(unsorted_vehicles_index):
					
					states.append(None)
				return states
			
			else:
				print("Length of the last frame is: ", FRAME_LENGTH)
				print("Rearranging is not possible in this case.. Requesting to merge")
				states.append(None) 
				return states

		# print("The value of N is:",  N)

		candidates = [[] for i in range(N_LANES)]
		for i in range(N_LANES):
			if lane_counter[i] > max_vehicle:				
				for j in unsorted_vehicles_index:
					l_s = vehicles[j].lane; l_d = vehicles[j].destination;
					if l_d == i+1 or ((i+1) > l_s and (i+1) < l_d) or ((i+1) > l_d and (i+1) < l_s):
						candidates[i].append(j)


		temp_supp_list = []
		for i in range(N_LANES):
			if lane_counter[i] > max_vehicle:
				difference = lane_counter[i] - max_vehicle	
				if difference > len(candidates[i]):
					print("difference: %r, len(candidates[%r]): %r, max_vehicle: %r, frame_len: %r"%(difference, i, len(candidates[i]), max_vehicle, FRAME_LENGTH ))
				for j in range(difference):
					# print("size of candidate list: ", len(candidates[i]))
					# print("difference: ", difference)
					temp_supp_list.append(candidates[i][j])

		for index in temp_supp_list:
			if index not in supporting_vehicles_index:
				supporting_vehicles_index.append(index)
		
		
		
		m = Model('lane_sorter')
		m.verbose = 0
		
		delta_pos = [m.add_var(lb = -2*FRAME_LENGTH, ub = 2*FRAME_LENGTH) for i in range(len(vehicles))]
		M = 3*FRAME_LENGTH
		binary_vars = [[] for v in range(len(vehicles))]
		for i in range(len(vehicles)):

			# Constraints to keep vehicles in the frame
			m += (positions[i] + delta_pos[i]) >= (LEN + SG/2)
			m += (positions[i] + delta_pos[i]) <= FRAME_LENGTH - SG/2

			current_lane = vehicles[i].lane
			dest_lane = vehicles[i].destination	
			# Add channel constraints for vehicles with no.(change in lane) = 2
			if abs(current_lane - dest_lane) == 2 and i not in supporting_vehicles_index:
				subindex = -1
				for j in range(i+1, len(vehicles)):				
					if vehicles[j].lane != current_lane:								
						subindex += 1
						binary_vars[i].append(m.add_var(var_type = BINARY))
						m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) + M * binary_vars[i][subindex] >= (LEN + SG)
						m += -1 * ((positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j])) + (1 - binary_vars[i][subindex]) * M >= (LEN + SG)

					# Constraints to keep vehicles in actual order in the lane
					if(vehicles[j].lane == current_lane):
						if vehicles[i].pos > vehicles[j].pos:
							m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) >= (LEN + SG)
						else:
							m += (positions[j] + delta_pos[j]) - (positions[i] + delta_pos[i]) >= (LEN + SG)

			# Add channel constraints for vehicles with no.(change in lane) = 1							
			if abs(current_lane - dest_lane) == 1 and i not in supporting_vehicles_index:
				subindex = -1
				for j in range(i+1, len(vehicles)):
					if (j not in sorted_vehicles_index and j not in supporting_vehicles_index)  or (j in sorted_vehicles_index or j in supporting_vehicles_index) and \
					(vehicles[j].lane == dest_lane) :
						subindex += 1
						binary_vars[i].append(m.add_var(var_type = BINARY))
						m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) + M * binary_vars[i][subindex] >= (LEN + SG)
						m += -1 * ((positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j])) + (1 - binary_vars[i][subindex]) * M >= (LEN + SG)

					# Constraints to keep vehicles in actual order in the lane
					if(vehicles[j].lane == current_lane):
						if vehicles[i].pos > vehicles[j].pos:
							m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) >= (LEN + SG)
						else:
							m += (positions[j] + delta_pos[j]) - (positions[i] + delta_pos[i]) >= (LEN + SG)

			# Add channel constraints for vehicles with no change in lane
			if current_lane == dest_lane or i in supporting_vehicles_index:
				subindex = -1			
				for j in range(i+1, len(vehicles)):								
					if j not in supporting_vehicles_index and (abs(vehicles[j].lane - vehicles[j].destination) == 2 or vehicles[j].destination == current_lane):
						subindex += 1
						binary_vars[i].append(m.add_var(var_type = BINARY))
						m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) + M * binary_vars[i][subindex] >= (LEN + SG)
						m += -1 * ((positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j])) + (1 - binary_vars[i][subindex]) * M >= (LEN + SG)			

					# Constraints to keep vehicles in actual order in the lane
					if(vehicles[j].lane == current_lane):
						if vehicles[i].pos > vehicles[j].pos:
							m += (positions[i] + delta_pos[i]) - (positions[j] + delta_pos[j]) >= (LEN + SG)
						else:
							m += (positions[j] + delta_pos[j]) - (positions[i] + delta_pos[i]) >= (LEN + SG)		
		
		# Using other variable list to convert abs function in the objective function
		delta_pos_prime = [m.add_var(lb = 0, ub = FRAME_LENGTH) for i in range(len(vehicles))]

		for i in range(len(delta_pos_prime)):
			m += delta_pos_prime[i] >= delta_pos[i]
			m += delta_pos_prime[i] >= -1*delta_pos[i]

		X_prime = m.add_var()
		m += xsum(delta_pos_prime[unsorted_vehicles_index[i]] for i in range(len(unsorted_vehicles_index))) \
			+ xsum(delta_pos_prime[sorted_vehicles_index[i]] for i in range(len(sorted_vehicles_index))) \
			+ xsum(delta_pos_prime[supporting_vehicles_index[i]] for i in range(len(supporting_vehicles_index))) <= X_prime		

		m += -1*xsum(delta_pos_prime[unsorted_vehicles_index[i]] for i in range(len(unsorted_vehicles_index))) \
			- xsum(delta_pos_prime[sorted_vehicles_index[i]] for i in range(len(sorted_vehicles_index))) \
			- xsum(delta_pos_prime[supporting_vehicles_index[i]] for i in range(len(supporting_vehicles_index))) <= X_prime
		
		m.objective = minimize(X_prime)	

		m.optimize()

		if m.num_solutions:			
			solution_found = True
			# print("Solution obtained................")
			if len(supporting_vehicles_index) == 0: all_vehicles_sorted = True
			
			
			
			for i in range(len(delta_pos)):
				if i not in supporting_vehicles_index:
					# print("VEHICLE POS: %r" %vehicles[i].pos)
					vehicles[i].pos = vehicles[i].pos + delta_pos[i].x
					
					veh_in_lane[vehicles[i].lane-1].remove(vehicles[i])
					vehicles[i].lane = vehicles[i].destination
					veh_in_lane[vehicles[i].lane-1].append(vehicles[i])

					
				else:
					vehicles[i].pos = vehicles[i].pos + delta_pos[i].x

			# return
			global k
			states.append(copy.deepcopy(vehicles))
			# return states
		else: 
			
			states.append(None)
			return states
			
	return states