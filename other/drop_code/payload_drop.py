# Created by: Joe Sanford
# Modified by: William Gregory
# Last Date: 31 October 2015
# Thank you Joe!

# Description: ArduPlane targeted payload drop

# ideal payload drop:
#	alt = 300 m
# 	groundspeed = 25 m/s
# ?? units ??



# Import
#-------------------------------------------------------------

#import sys
import math
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink") # includes the Utilities class
import MAVLink

# mav and waypoint
idmavcmd = MAVLink.MAV_CMD.WAYPOINT
id = int(idmavcmd)

# Variables
#-------------------------------------------------------------

## Constants
gravity_metric = 9.807 			# gravity on Earth (m/s^2)							(constant)
gravity_us = 32.2				# (ft/s^2)											(constant)
gravity_us2 = 21.937 			# (mph/s)											(constant)
radius_earth = 6371000 			# meters											(constant)

target_location = [39.40456, -119.761292] # lat lon of drop target

## Settings
flight_units = 'M'				# M (metric), US (us units)
use_wind = False					# True to use wind in drop calculation
default_alt = 100				# default altitude (feet)
# -drop
real_drop = True
drop_type = 2					# 1: time, 2: distance
target_waypoint = 6				# waypoint to perform drop	--NOTE make this an array for multiple drops
countdown_range = 50			# countdown range in addition to release_distance
# -- drop type 1
# -- drop type 2
distance_margin = 15			# distance before drop type 2 countdown

## State Variables
mode = 0						# current mode 0: idle, 1:run, 2:done
abort = False					# set true to abort flight and RTL
retry_drop = False				# true when drop failed.  retry

drop_location = []
wind_drop_location = []

# -current flight data
current_altitude = 0.0 			# altitude
current_groundspeed = 0.0 		# groundspeed
current_groundcourse = 0.0
current_distance = 0.0 			# distance to waypoint
current_waypoint = 0			# waypoint number
current_wind_speed = 0.0		# wind velocity
current_wind_direction = 0.0	# wind direciton

# -drop data
payload_dropped = False			# true once payload dropped. reset on new run
fall_time = 0.0					# time for payload to fall from altitude 			(calculated)
release_distance = 0.0			# distance from target to payload drop point 		(calculated)
distance_before_release = 0.0	# distance from current location to payload drop 	(calculated)
time_before_release = 0.0		# time in seconds until release 					(calculated)
time_of_release = 0.0			# sceduled time of release 							(calculated)



# General Funcitons
#-------------------------------------------------------------

# get updated flight data
def getFlightData():
	global current_time, current_distance, current_altitude, current_groundspeed, current_groundcourse, current_waypoint, current_wind_direction, current_wind_speed
	current_time = time.time()
	current_distance = cs.wp_dist
	current_altitude = cs.alt
	current_groundspeed = cs.groundspeed
	current_groundcourse = cs.groundcourse
	current_waypoint = cs.wpno
	current_wind_direction = cs.wind_dir
	current_wind_speed = cs.wind_vel

# print flight data
def printFlightData():
	print("")
	print("Flight Data - " + str(current_time))
	print("current_altitude: " + str(current_altitude))
	print("current_distance: " + str(current_distance))
	print("current_groundspeed: " + str(current_groundspeed))
	print("current_groundcourse: " + str(current_groundcourse))
	print("current_waypoint: " + str(current_waypoint))
	print("current_wind_direction: " + str(current_wind_direction))
	print("current_wind_speed: " + str(current_wind_speed))
	print("")
	
# create and set new waypoint
def setNewWaypoint(wp_lat,wp_lng,wp_alt):
	'''
	wp = Locationwp().Set(wp_lat,wp_lng,wp_alt, id)
	wp_num = target_waypoint
	MAV.setWP(wp, wp_num, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT)
	MAV.setWPCurrent(wp_num)
	MAV.setWPACK()
	'''
	
	new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
	MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,wp_lat)		# set waypoint latitude
	MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,wp_lng)		# set waypoint longitude
	MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,wp_alt)		# set waypoint altitude
	MAV.setGuidedModeWP(new_wp)											# fly to the new waypoint
	
	
	#MAV.doCommand(MAVLink.MAV_CMD.WAYPOINT, 0, 0.000621371, 0, 0, wp_lat, wp_lng, wp_alt)
	#MAV.doCommand(MAVLink.MAV_CMD.DO_DIGICAM_CONTROL, 0, 0, 0, 0, 0, 0, 0);
	
def abortRun():
	global real_drop, retry_drop
	real_drop = False
	retry_drop = True
	
# wind functions
#-------------------------------------------------------------

def findWindComp(wind_vel, wind_dir, fall_time):

	# Description: break the wind velocity (m/s) and direciton (deg) into components X and Y.
	# Corect the sign based on direction.  Multliply by fall_time to get displacement over the fall.
	# Convert this back into a magnitutde and direction
	
	# get x,y components of wind
	wind_x = wind_vel*math.cos(wind_dir)
	wind_y = wind_vel*math.sin(wind_dir)
	
	# check sign
	if 0 <= wind_dir < 90: # quadrant 1 (+,+)
		pass
	elif 90 <= wind_dir < 180: # quadrant 2 (-,+)
		wind_x = wind_x * (-1)
	elif 180 <= wind_dir <= 270: # quadrant 3 (+,+)
		wind_x = wind_x * (-1)
		wind_y = wind_y * (-1)
	elif 270 <= wind_dir <= 360: # quadrant 4 (+,-)
		wind_y = wind_y * (-1)
	
	# find displacement
	disp_x = wind_x*fall_time
	disp_y = wind_y*fall_time
	
	## no y (head wind) 
	#disp_y = 0
	
	# convert displacement from wind into direciton and magnitutde
	disp_wind_mag = math.sqrt(disp_x**2 + disp_y**2)
	disp_wind_dir = math.atan(disp_y/disp_x)
	
	return disp_wind_mag, disp_wind_dir
	
def findNewCoord(start_lat, start_lon, move_mag, move_dir):
	
	# Description: return new latitute and longitute given a starting latitute, longitute, displacement magnitutde and direciton
	# INFO: https://www.physicsforums.com/threads/calculating-lat-long-with-range-bearing.605218/
	
	delta_n = move_mag*math.sin(450-move_dir) # delta north
	delta_e = move_mag*math.cos(450-move_dir) # delta east
	
	delta_lat = 360*delta_n/(2*math.pi*radius_earth) # delta latitute
	delta_lon = 360*delta_e/(2*math.pi*radius_earth*math.cos(delta_lat)) # delta longitute
	
	new_lat = start_lat + delta_lat
	new_lon = start_lon + delta_lon
	
	return new_lat, new_lon
	
# autorun
def findDropLocation():
	# returns drop latitude and longitude based on groundcourse to target
	global drop_location
	drop_location = findNewCoord(target_location[0], target_location[1], release_distance, (180 - current_groundcourse))
	print("target: " + str(target_location[0]) + " : " + str(target_location[1]))
	print("drop: " + str(drop_location[0]) + " : " + str(drop_location[1]))

	
def newDropWithWind():
	# returns drop latitude and longitude based on current wind conditions
	global wind_drop_location
	wind_disp, wind_dir = findWindComp(current_wind_speed, current_wind_direction, fall_time)
	drop_lat, drop_lon = findNewCoord(drop_location[0], drop_location[1], wind_disp, wind_dir)
	wind_drop_location = drop_lat, drop_lon
	print("wind drop: " + str(wind_drop_location[0]) + " : " + str(wind_drop_location[1]))
	
	
def checkDropParam():
	# if out_of_param: abort()
	pass

# Drop Functions
#-------------------------------------------------------------

def printDropData(val):
	print("fall time: " + str(fall_time))
	print("release distance: " + str(release_distance))
	if val == 1:
		print("distance_before_release: " + str(distance_before_release))
		print("time_before_release: " + str(time_before_release))
		print("time_of_release: " + str(time_of_release))

# calculate drop
def calculateDrop(val):
	global fall_time, release_distance, distance_before_release, time_before_release, time_of_release
	getFlightData();
	if flight_units == 'US':
		fall_time = math.sqrt((2*current_altitude)/(gravity_us)) #calculate fall time based on current alt
	if flight_units == 'M':
		fall_time = math.sqrt((2*current_altitude)/(gravity_metric)) #calculate fall time based on current alt	
	release_distance = current_groundspeed*fall_time #calculate release distance for bomb
	if val == 1:
		distance_before_release = current_distance - release_distance
		time_before_release = distance_before_release/current_groundspeed
		time_of_release = time.time() + time_before_release

# Drop type 1
def dropTypeOne():
	while payload_dropped != True:
		Script.Sleep(time_before_release)
		dropPayload(); # drop payload

# Drop type 2
def dropTypeTwo():
	while payload_dropped != True:
		calculateDrop(1);
		if distance_before_release <= distance_margin:
			print("Countdown Start")
			printDropData(1)
			Script.Sleep(time_before_release)
			dropPayload(); # drop payload

# release the payload
def dropPayload():
	global payload_dropped
	if payload_dropped == False and abort == False:
		print ("bomb away")
		if real_drop:
			Script.SendRC(7, 953, True)
			payload_dropped = True
			Script.Sleep(5000)
			Script.SendRC(7, 2028, True)
		# reset waypoint
		Script.ChangeMode("AUTO")


# Drop Manager
#-------------------------------------------------------------

def dropManager():
	global mode, abort
	getFlightData();
	# -on payload drop run
	if current_waypoint == target_waypoint:	
		print("On Run")
		Script.Sleep(1000) # align to waypoint
		calculateDrop(1); # calculate
		findDropLocation() # drop location - once
		setNewWaypoint(drop_location[0], drop_location[1], default_alt) # no wind drop
		if use_wind:
			newDropWithWind() # wind drop location
			setNewWaypoint(wind_drop_location[0], wind_drop_location[1], default_alt)
		while abort != True and mode == 0:
			calculateDrop(1); # calculate
			printFlightData() # print
			# -not in range
			if current_distance > release_distance + countdown_range: 
				print("Approaching Release Point")
			# -within drop range
			else:	
				mode = 1 # mode 1
				print("Starting drop type: " + str(drop_type))
				print("")
				if drop_type == 1:
					dropTypeOne(); # d1
				elif drop_type == 2:
					dropTypeTwo(); # d2
				else: abort = True
				mode = 2 # mode 2 - mission complete

# Autostart
#-------------------------------------------------------------

def autostart():
	print("payload drop started")
	print("")
	Script.ChangeMode("AUTO")
	while abort != True:
		dropManager();
		if mode == 2:
			print("Mission Complete")
			break
			
	#Script.ChangeMode('RTL')
	if abort: return False
	else: return True
	
autostart();
