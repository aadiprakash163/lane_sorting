from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import time
import matplotlib.pyplot as plt
import traci
from algo import sort_vehicles
from queue import Queue
import threading
from statistics import mean, median, mode

COLOR_MAP = {
	0: "0,1,0",
	1: "1,0,0",
	2: "0,0,1"
}
REV_COLOR_MAP = {
	(0,255,0,255): 0,
	(255,0,0,255): 1,
	(0,0,255,255): 2
}

SORT_DIST = {}
IN_TIME = {}
STEP = 0
STEP_LEN = 0.1
def sort_vehicles_thread(q, config, frame_len, v_len, new_frame):
	q.put(sort_vehicles(config, frame_len, v_len, new_frame))
	return

def traci_get_veh_pos(vid, vlen): 
	return (traci.vehicle.getPosition(vid)[0])
	# return (traci.vehicle.getPosition(vid)[0] - vlen)


class Vehicle:
	def __init__(self, id, lane, destination, pos):
		self.id = id
		self.lane = lane
		self.destination = destination
		self.pos = pos

class Frame:
	def __init__(self,base,frame_len,step_len,vehicleIds,vehicle_dest_lanes,base_velocity,veh_len, front):
		self.base = base
		self.front = front
		self.frame_len = frame_len
		self.step_len = step_len
		self.vehicleIds = vehicleIds
		self.base_velocity = base_velocity
		self.vehicle_dest_lanes = vehicle_dest_lanes
		self.veh_len = veh_len

		self.thread_started = False
		self.thread_val_queue = Queue()
		self.needs_merge = False
		self.states = None	
		self.is_sorted = False
		self.just_formed = True
		

	def get_vehicle_pos(self, vid) : 
		return traci_get_veh_pos(vid, self.veh_len)

	def get_config(self) :
		vehicle_config = []
		for v in self.vehicleIds : 
			vehicle_id = v;
			vehicle_lane =  traci.vehicle.getLaneIndex(v) + 1;
			vehicle_destination = self.vehicle_dest_lanes[v];			
			vehicle_pos = self.get_vehicle_pos(v) - self.base;
			veh = Vehicle(vehicle_id, vehicle_lane, vehicle_destination, vehicle_pos)			
			vehicle_config.append(veh)
		return vehicle_config
	
	def tick(self):
		self.base += self.base_velocity * self.step_len
		

	def move_to_pos(self, vehicle_id,dist):
		dist_off = (float(dist)  + float(self.base) - float(self.get_vehicle_pos(vehicle_id)))
		speed_off = self.base_velocity - float(traci.vehicle.getSpeed(vehicle_id))
		updated_vel = traci.vehicle.getSpeed(vehicle_id) + 0.075 * dist_off + 0.1 * speed_off
		vel_set = max(0. , updated_vel)
		if abs(dist_off) < 0.01 :
			traci.vehicle.setSpeed(vehicle_id, self.base_velocity)
			return True
		else:
			traci.vehicle.setSpeed(vehicle_id, speed=vel_set)
			return False


	def move_vehicle(self, vehicleId, dist) :
		unaligned = False
		if not unaligned :
			unaligned = self.move_to_pos(vehicleId, dist)
			
		if not unaligned :
			return False
		return True

	def stabilize(self):
		for vid in self.vehicleIds : 
			traci.vehicle.setSpeed(vid, self.base_velocity)

	def sort(self) :
		global STEP
		if self.thread_started : 

			if self.states :

				if len(self.states) > 0 :
					curr_state = self.states[0]
					done = True
					
					if curr_state == None :
						self.needs_merge = True
					else : 
						for v in curr_state :
							res = self.move_vehicle(v.id, v.pos)
							done = done and res
							
						if done:
							
							for v in curr_state :
								if v.lane == v.destination:
									
									if SORT_DIST[v.id] == 0:
										SORT_DIST[v.id] = STEP - IN_TIME[v.id]
									traci.vehicle.changeLane(v.id, v.lane -1, 1)
									
									
							state_achieved = True
							for v in curr_state :
								if v.lane != traci.vehicle.getLaneIndex(v.id) + 1:
									state_achieved = False
									break

							if state_achieved :
								self.states.remove(self.states[0])

				if len(self.states) == 0 :
					self.is_sorted = True
			else : 
				if not self.thread_val_queue.empty() :
					self.states = self.thread_val_queue.get()		
		else :
			tid = threading.Thread(target=sort_vehicles_thread, args=(self.thread_val_queue, self.get_config(), self.frame_len, self.veh_len, self.just_formed))
			tid.start()
			self.just_formed = False
			self.thread_started = True
	

class Scenario:
	
	def __init__(self, no_vehicles, base_velocity, frame_len, veh_len, veh_density, step_len, safety_gap):
		self.frame_len = frame_len
		self.no_vehicles = no_vehicles
		self.step_len = step_len
		self.base_velocity = base_velocity
		self.veh_density = veh_density
		self.veh_len = veh_len
		self.safety_gap = safety_gap
		
		self.traci_port = random.randint(2000,9000)
		
		self.sub_process = None
		self.time = 0
		self.vehicle_ids = []
		self.base_pos = 0
		self.frames = []
		self.all_vehicles = []
		self.vehicle_curr_lanes = {}
		self.vehicle_dest_lanes = {}
		self.last_vehicle_dispatch_time = 0
		self.sort_time = {}
		

	def traci_init(self):
		traci.init(self.traci_port)

	def generate_routefile(self):
		
		N = 3600 # number of time steps
		
		with open("sumo/cross.rou.xml", "w") as routes:
			print("""
			<routes>
				<vType id="type_1" accel="1000" decel="1000" sigma="0.0" length="%d" width = "1.5" minGap="0.0" minGapLat = "0.0" maxSpeed="1000" guiShape="passenger"/>
				
				<route id="route_1" edges="lane_i lane_o" />
			"""%(self.veh_len), file=routes)

			vehicles_added = 0
			delay = 0
			
			vehicles = []
			while (vehicles_added < self.no_vehicles) : 
				# lane = random.randint(0,2)
				p_lane_1 = 1. /self.veh_density
				p_lane_2 = 1. /self.veh_density
				p_lane_3 = 1. /self.veh_density
				

				if (random.uniform(0, 1) < p_lane_1) and (vehicles_added < self.no_vehicles) :
					go_to_lane = random.randint(0,2)
					vid = """V%d"""%vehicles_added

					self.vehicle_curr_lanes[vid] = 1
					self.vehicle_dest_lanes[vid] = go_to_lane + 1
					vehicles.append(Vehicle(vid,1,go_to_lane,delay + random.uniform(0,1)))
					vehicles_added += 1
				
				if (random.uniform(0, 1) < p_lane_2) and (vehicles_added < self.no_vehicles) :
					go_to_lane = random.randint(0,2)
					vid = """V%d"""%vehicles_added

					self.vehicle_curr_lanes[vid] = 2
					self.vehicle_dest_lanes[vid] = go_to_lane + 1
					vehicles.append(Vehicle(vid,2,go_to_lane,delay + random.uniform(0,1)))
					vehicles_added += 1
				
				if (random.uniform(0, 1) < p_lane_3) and (vehicles_added < self.no_vehicles) :
					go_to_lane = random.randint(0,2)
					vid = """V%d"""%vehicles_added

					self.vehicle_curr_lanes[vid] = 3
					self.vehicle_dest_lanes[vid] = go_to_lane + 1
					vehicles.append(Vehicle(vid,3,go_to_lane,delay + random.uniform(0,1)))
					vehicles_added += 1

				delay += 2

			vehicles = sorted(vehicles, key = lambda v: v.pos)
			for v in vehicles : 
				print(""" <vehicle id="%s" type="type_1" route="route_1" depart="%f" departSpeed="%f" departLane= "%d" color="%s"/>" """ %(v.id,v.pos,self.base_velocity,self.vehicle_curr_lanes[v.id]-1, COLOR_MAP[self.vehicle_dest_lanes[v.id]-1]), file=routes)
			print("</routes>", file=routes)
		

	def get_vehicle_pos(self, vid) : 
		return traci_get_veh_pos(vid, self.veh_len)

	def set_initial_speed(self):
		# min_pos = 2*self.frame_len
		global STEP
		self.time += self.step_len
		vehicles_dept = traci.simulation.getDepartedIDList()
		
		if len(vehicles_dept) > 0 :
			self.last_vehicle_dispatch_time = self.time

		for v in vehicles_dept :	
			IN_TIME[v] = STEP	
			SORT_DIST[v] = 0
			traci.vehicle.changeLane(v, traci.vehicle.getLaneIndex(v), 1)
			traci.vehicle.setSpeed(v,self.base_velocity)
			traci.vehicle.setSpeedMode(v, 0)
			traci.vehicle.setLaneChangeMode(v, 256)

		
			self.vehicle_curr_lanes[v] = traci.vehicle.getLaneIndex(v) + 1
			self.vehicle_dest_lanes[v] = REV_COLOR_MAP[traci.vehicle.getColor(v)]+1
			self.vehicle_ids.append(v)
			self.all_vehicles.append(v)
			self.sort_time[v] = None
		
	def sumo_start(self):
		self.sub_process = subprocess.Popen(['sumo-gui', "-c", "sumo/cross.sumocfg", "--step-length", str(self.step_len), "--fcd-output", "fcd_output.xml", "--remote-port", str(self.traci_port)], stdout=sys.stdout, stderr=sys.stderr)
	
	def sumo_wait(self):
		self.sub_process.wait()


	def farthest_vehicle(self) : 
		vehicle_dist_list = list(map(lambda v_id: self.get_vehicle_pos(v_id), self.vehicle_ids))
		vehicle_dist_list.append(0)
		return max(vehicle_dist_list)

	def get_min_pos(self):
		vehicle_poses = list(map(lambda v_id: self.get_vehicle_pos(v_id), self.vehicle_ids))
		
		return(min(vehicle_poses))



	def get_dist_data(self) : 
		
		vehicle_sort_time = list(map(lambda v_id: SORT_DIST[v_id] , self.all_vehicles))
		print(vehicle_sort_time)
		return (mean(vehicle_sort_time ) * STEP_LEN, max(vehicle_sort_time)*STEP_LEN)

	def is_all_sorted(self) :
		if len(self.all_vehicles) != self.no_vehicles :
			return False
		for v in self.all_vehicles :
			if traci.vehicle.getLaneIndex(v) + 1 == self.vehicle_dest_lanes[v] and self.sort_time[v] == None :
				# self.sort_time[v] = self.time
				self.sort_time[v] = self.get_vehicle_pos(v)

		for v in self.all_vehicles :
			if  traci.vehicle.getLaneIndex(v) + 1 != self.vehicle_dest_lanes[v] :
				return False
		return True

	def run(self):
		self.traci_init()
		
		x = True
		
		q = Queue()

		states = None
		global STEP

		while traci.simulation.getMinExpectedNumber() > 0: 
			STEP = STEP + 1
			traci.simulationStep()
			
			self.set_initial_speed()
			min_pos = 3.10
			farthest_veh = self.farthest_vehicle()
			if farthest_veh >= self.frame_len + min_pos : 				
				# min_pos = self.get_min_pos()
				
				
				if len(self.frames)> 0:					
					new_front = min_pos + self.frame_len
					
					self.frames.append(Frame(min_pos,new_front - min_pos,self.step_len,self.vehicle_ids,self.vehicle_dest_lanes,self.base_velocity,self.veh_len, new_front))				
					print("=========================================================")
					print("One frame CREATED, number of frames in simulation is: ", len(self.frames))
					print("This frame front: %r, previous frame base: %r, last_veh_pos: %r"%(self.frames[-1].front, self.frames[-2].base, min_pos))
				else:					
					self.frames.append(Frame(min_pos,self.frame_len,self.step_len,self.vehicle_ids,self.vehicle_dest_lanes,self.base_velocity,self.veh_len, min_pos + self.frame_len))
					print("=========================================================")
					print("One frame CREATED, number of frames in simulation is: ", len(self.frames))

				
				self.vehicle_ids = []
			
			for i in range(len(self.frames)):
				frame = self.frames[i]
				

			updated_frames = self.frames
			# new_length = len(self.frames)

			for i in range(len(self.frames)):
				frame = self.frames[i]
				
				if frame.needs_merge == True:
					# print("Frame %r is to be MERGED."%(i+1))						
					if i == 0 : 					
						# print("Frame %r is to be MERGED."%(i+1))						
						fr_base = frame.base
						fr_len = frame.frame_len + self.frame_len/2							
						fr_vehs = frame.vehicleIds
						updated_frames[0] = Frame(fr_base,fr_len,self.step_len,fr_vehs,self.vehicle_dest_lanes,self.base_velocity,self.veh_len, fr_base + fr_len)
						print("Length of frame number %r is %r: "%(i+1, fr_len))
					else :						
						frame_ahead = self.frames[i-1]
						
						
						if frame_ahead.is_sorted:
							fr_base = frame.base
							fr_vehs = frame_ahead.vehicleIds
							fr_len = frame_ahead.base - frame.base +  frame_ahead.frame_len 
							# print("Length of frame number %r is %r: "%(i+1, fr_len))
							fr_vehs.extend(frame.vehicleIds)
							updated_frames.remove(frame_ahead)
							updated_frames[i-1] = Frame(fr_base,fr_len,self.step_len,fr_vehs,self.vehicle_dest_lanes,self.base_velocity,self.veh_len, fr_base + fr_len)
							print("Frames merged.....")
							break

			self.frames = updated_frames
			for i in range(len(self.frames)):
				
				self.frames[i].tick()
				self.frames[i].sort()
				
				
			
			if self.is_all_sorted() :

				traff_density = float(self.no_vehicles) / float(self.last_vehicle_dispatch_time)
				mean_dis, max_dist = self.get_dist_data()

				
				self.sub_process.kill()

				return (traff_density, mean_dis, max_dist)

			
				