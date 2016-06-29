# Created by: William Gregory
# Date: 6th November 2015
# Version: 0.1
# Description: ArduPilot Plane autopilot python interaction and mission manager
# Note:

# Information about ArduPilot python commands:
# --------------------------------------------------------------------------------------
# cs.???? = currentstate, any variable on the status tab in the planner can be used.
# -some varialbes: roll, pitch, yaw, lat, lng, groundcourse, alt, groundspeed, wp_dist, wpno, mode, armed, battery_remaining
# -more info here: http://planner.ardupilot.com/wiki/using-python-scripts-in-mission-planner/
#
# Script.????
# Script.Sleep(ms) - sleep time in milliseconds
# Script.ChangeParam(name,value) - change a parameter value
# Script.GetParam(name) - read a parameter value
# -parameter list here: http://plane.ardupilot.com/wiki/arduplane-parameters/
# Script.ChangeMode(mode) - ex. AUTO, RTL, AUTO, FBWA, FBWB, LOITER
# -mode list here: http://plane.ardupilot.com/wiki/flight-modes/
# Script.WaitFor(string,timeout)
# Script.SendRC(channel,pwm,sendnow) - set servo to position
#
# MAV.doCommand(command);  - MAVLink Mission Command Messages
# -command messages here: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/
#
# RC Input and Output values - http://dev.ardupilot.com/wiki/learning-ardupilot-rc-input-output/


# Import
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

import math
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp


# Settings
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

# general
enable = True # set true to enable script
debug_print = True # output general flight data
default_mode = 'RTL' # default mode to goto if abort or mission complete
# mission
missions = [1] # list of missions for flight - 1: payload drop
mission_wps = [5] # list of waypoints coresponding each 'mission'
#					Home(0)						Payload Drop(1)
known_locations = [[39.404778, -119.761238], [39.40456, -119.761292]] # common lat and lng values
# servo channels
channel_1 = 1 # ailerons - roll
channel_2 = 2 # elevator - pitch
channel_3 = 3 # throttle - altitude
channel_4 = 4 # rudder   - yaw
channel_5 = 5


# State Variables
# --------------------------------------------------------------------------------------

# general
current_mission = 0				# current mission loaded form missions[]
current_mission_waypoint = 0	# from mission_wps[]
current_mission_index = 0		# current index of missions[]
current_mission_state = 0		# current mode - 0: idle, 1:run, 2:done
abort = False					# set true to abort flight and RTL
# current flight data
current_time = 0
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


# Script Menu Interface
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

def MainMenu():
	print("main menu")
	print("")
	print("1: payloadDrop")
	print("2: flight maneuvers")
	print("3: op")
	print("")
	choice_temp = raw_input("enter choice: ")
	# mission stuff - ex. start custom mission, run simulation(ex. abort)
	if choice_temp == '1':
		pass
	# plane functions - ex. goto custom location, report info
	elif choice_temp == '2':
		pass
	# settings - ex. 
	elif choice_temp == '3':
		pass
	#
	else:
		print("")
		print("that is not an option. try again")
		print("")
		MainMenu()


# Flight Functions
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

# get updated flight data
def getFlightData():
	# add check for time it takes to get this data
	global current_distance, current_altitude, current_groundspeed, current_waypoint, \
		current_wind_direction, current_wind_speed, current_roll, current_pitch, \
		current_yaw, current_lat, current_lng
	current_time = time.time()
	current_waypoint = cs.wpno
	current_mode = cs.mode
	current_altitude = cs.alt
	current_groundspeed = cs.groundpspeed
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
	print("FLIGHT DATA") # " + str(current_time))
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
	
# create and set new waypoint
def setNewWaypont(wp_lat,wp_lng,wp_alt):
	new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
	MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,wp_lat)		# set waypoint latitude
	MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,wp_lng)		# set waypoint longitude
	MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,wp_alt)		# set waypoint altitude
	MAV.setGuidedModeWP(new_wp)											# fly to the new waypoint

# abort mission and return home (error encountered)
def abortFlight():
	Script.ChangeMode(RTL) # set mode to RTL
	enable = False # disable script control


# Mission Functions
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

# load next mission value
def setMission(val):
	# increment mission index if val is 1
	# only 0 on startup
	if val == 1: current_mission_index += 1
	current_mission = missions[current_mission_index]
	current_mission_waypoint = mission_wps[current_mission_index]

def startMission():
	if current_mission == 1:
		payloadDrop(); # execute payload drop
	else:
		# error unkown mission type
		print("ERROR: unknown mission type")
		abort = True

# mission complete return home (clean exit)
def missionComplete():
	print("mission complete")
	mode = 2

# Function Managers
# --------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------

# manage general flight operations
def FlightManager():
	# check if it is safe for mission execution
	# flight path manager with waypoints
	getFlightData();
	if debug_print: printFlightData();
	

def MissionManager():
	if mode == 2:
		setMission(1); # set next mission
		mode = 0
	# set mission paramters
	if current_mission_waypoint == current_waypoint:
		mode = 1
		startMission(); # execute mission
	# close mission


# Autostart
# --------------------------------------------------------------------------------------

# code to run on script start
def firstStartSetup():
	setMission(0);

# manage script start
def autostart():
	atempt_goto_waypoint = False
	if atempt_goto_waypoint: setNewWaypont();
	while enable:
		getFlightData();
		printFlightData();
	# firstStartSetup();
	# while enable:
		# FlightManager();
		# MissionManager();

autostart();