# Created by: William Gregory
# Date: 10th November 2015
# Description: ArduPilot print current flight info

import clr
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp

# current flight data
current_waypoint = 0			# waypoint number
current_mode = ''				# autopilot mode
current_altitude = 0.0 			# altitude
current_groundspeed = 0.0 		# groundspeed
current_wp_distance = 0.0 		# distance to waypoint
current_wind_speed = 0.0		# wind velocity
current_wind_direction = 0.0	# wind direciton
current_roll = 0.0				# roll
current_pitch = 0.0				# pitch
current_yaw = 0.0				# yaw
current_lat = 0.0				# latitude
current_lng = 0.0				# longitude

# get updated flight data
def getFlightData():
	# add check for time it takes to get this data
	global current_distance, current_altitude, current_groundspeed, current_waypoint, \
		current_wind_direction, current_wind_speed, current_roll, current_pitch, \
		current_yaw, current_lat, current_lng
	current_waypoint = cs.wpno
	current_mode = cs.mode
	current_altitude = cs.alt
	current_groundspeed = cs.groundspeed
	current_wp_distance = cs.wp_dist
	current_wind_direction = cs.wind_dir
	current_wind_speed = cs.wind_vel
	current_roll = cs.roll
	current_pitch = cs.pitch
	current_yaw = cs.yaw
	current_lat = cs.lat
	current_lng = cs.lng

# print flight data
def printFlightData():
	print("FLIGHT DATA")
	print("current_waypoint: " + str(current_waypoint))
	print("current_mode: " + str(current_mode))
	print("current_altitude: " + str(current_altitude))
	print("current_groundspeed: " + str(current_groundspeed))
	print("current_wp_distance: " + str(current_wp_distance))
	print("current_wind_direction: " + str(current_wind_direction))
	print("current_wind_speed: " + str(current_wind_speed))
	print("current_roll: " + str(current_roll))
	print("current_pitch: " + str(current_pitch))
	print("current_yaw: " + str(current_yaw))
	print("current_latitude: " + str(current_lat))
	print("current_longitude: " + str(current_lng))
	print("")
	
# autorun
while True:
	getFlightData()
	printFlightData()
	Script.Sleep(1000)