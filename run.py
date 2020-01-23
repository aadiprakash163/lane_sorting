#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import time
import matplotlib.pyplot as plt
import csv

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

from scenario import Scenario

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options



def generate_result() :
    step_len = 0.1
    no_vehicles = 50

    #Change the Common Velocity from here
    base_velocity = 5

    #Change the vehicle length from here
    veh_len = 3.0
    
    # Change the number of vehicles in a frame from here
    no_veh_in_lane = 1


    frame_len = float(no_veh_in_lane) * veh_len * 5/3
    safety_gap = float(veh_len *2 /3)
    print(frame_len)

    # Change the value of K to genereate route files corresponding to different traffic densities
    K = 5

    scenario = Scenario(
        no_vehicles=no_vehicles,
        base_velocity=base_velocity,
        frame_len=frame_len,
        veh_len=veh_len,
        veh_density=K,
        step_len=step_len,
        safety_gap = safety_gap
        )
            
    # Uncomment following lines to generate the route file
    # scenario.generate_routefile()
    # exit()
    
    scenario.sumo_start()
    val = scenario.run()
    scenario.sumo_wait()

    veh_density, mean_dis, max_dist = val
    veh_density *= 3600 # vehicle density in vehicles / hour
    
    
    print("here are the results\n ++++++++++++++++++++++++++++++++++++++++++\n")
    print([str(mean_dis), str(max_dist)])




if __name__ == "__main__":
    options = get_options()
    generate_result()

