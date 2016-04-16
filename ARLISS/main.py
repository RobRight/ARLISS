#!/usr/bin/python

# ARLISS Quadrotor Mission Code

# History:
# - WG 03/03/2016: create file and inital code
# - WG 04/15/2016: remove quad class. rewrite mission.main_run

# Note:
#	- all distance in meters (m)
#	- positions [lat,lon]
#	- Organized as follows:
#		-Mission
#			-Move
#			-Sensors
#
# Add:
#	- way to arm/disarm quad
#	- detect launch
#	- recovery
#	- waypoint navigation
#	- landing

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
# Script.SendRC(channel,pwm,sendnow) - set servo to pos ition
#
# MAV.doCommand(command);  - MAVLink Mission Command Messages
# -command messages here: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/
#
# RC Input and Output values - http://dev.ardupilot.com/wiki/learning-ardupilot-rc-input-output/
# --------------------------------------------------------------------------------------

#import sys
#import math
#import time
import clr
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink") # includes the Utilities class
import MAVLink

#
# Move Class
# ------------------------------------------------------
#

class Move:
	merror = False # true if there is a move related error
	# init
	def __init__(self):
		print("movement online")
	# set new waypoint with altitude
	def set_waypoint(target):
		#MAV.doCommand(MAVLink.MAV_CMD.WAYPOINT, 0, 0.000621371, 0, 0, wp_lat, wp_lng, wp_alt) - NODE: do not know if this command works
		new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
		MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,wp_lat)		# set waypoint latitude
		MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,wp_lng)		# set waypoint longitude
		MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,wp_alt)		# set waypoint altitude
		MAV.setGuidedModeWP(new_wp)											# fly to the new waypoint
	# call to land
	def start_landing():
		# http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_land
		# MAV_CMD_NAV_LAND(0,0,0,0,l,l) - NODE: do not know if this command works
		Script.ChangeMode('LAND') # set mode to LAND - http://copter.ardupilot.com/wiki/land-mode/

#
# Sensors Class
# ------------------------------------------------------
#

class Sensors:
	current_time = 0
	current_mode = ''
	current_distance = 0
	current_waypoint = 0
	current_altitude = 0.0
	current_groundspeed = 0.0
	current_groundcourse = 0.0
	current_wind_direction = 0.0
	current_wind_speed = 0.0
	current_roll = 0.0
	current_pitch = 0.0
	current_yaw = 0.0
	current_lat = 0.0
	current_lng = 0.0
	
	# init
	def __init__(self):
		print("sensors online")
	
	# get current flight data
	def get_flight_data():
		global current_time, current_mode, current_distance, current_altitude, \
			current_groundspeed, current_groundcourse, current_waypoint, \
			current_wind_direction, current_wind_speed, current_roll, \
			current_pitch, current_yaw, current_lat, current_lng
		current_time = time.time()
		current_mode = cs.mode
		current_distance = cs.wp_dist
		current_waypoint = cs.wpno
		current_altitude = cs.alt
		current_groundspeed = cs.groundspeed
		current_groundcourse = cs.groundcourse
		current_wind_direction = cs.wind_dir
		current_wind_speed = cs.wind_vel
		current_roll = cs.roll
		current_pitch = cs.pitch
		current_yaw = cs.yaw
		current_lat = cs.lat
		current_lng = cs.lng
	
	# output current flight data
	def output():
		print("sensors current:")
		print("- time: " + str(current_time))
		print("- distance: " + str(current_distance))
		print("- altitude: " + str(current_altitude))
		print("- groundspeed: " + str(current_groundspeed))
		print("- groundcourse: " + str(current_groundcourse))
		print("- waypoint: " + str(current_waypoint))
		print("- wind_direction: " + str(current_wind_direction))
		print("- wind_speed: " + str(current_wind_speed))

#
# Mission Class
# ------------------------------------------------------
#

class Mission:
	mov = Move()
	sen = Sensors()
	
	# --- SET BEFORE MISSION ---
	target_pos = [31.002, -110.010]
	recovery_start_alt = 1500.0
	landing_start_alt = 20.0
	landing_start_dist = 10.0
	# --- SET BEFORE MISSION ---
	
	# state variables
	launch_pos = [0.0, 0.0]
	launch_alt = 0
	ejected_pos = [0.0, 0.0]
	rocket_launched = False # true once rocket launched
	ejected = False # true once released from rocket
	recovered = False # true once in stable flight
	landed = False
	armed = False # set to True to engage system
	
	def __init__(self):
		print("mission online")
	
	# call at mission start
	def start():
		flight_mode = 0
		launch_pos = [sen.current_lat, sen.current_lon]
		launch_alt = sen.current_altitude
	
	# fly to waypoint
	def directed_flight():
		mov.set_waypoint(target_pos, alt)
	
	# ------------
	# mission code
	# ------------
	def main_run():
		start()
		# test until rocket launched
		while rocket_launched == false:
			if sen.current_altitude>start_alt+1000: rocket_launched = True
		print("rocket launched")
		
		# test until rocket ejected
		while ejected == false:
			if sen.current_altitude>max_alt: max_alt = sen.current_altitude
			if sen.current_altitude<max_alt+200: ejected = True # NOTE: maybe use accelerometer
		print("quad ejected")
		ejected_pos = [sen.curren_lat, sen.curren_lon]
		
		# wait until recovery start alt
		while sen.current_altitude>recovery_start_alt:
			# recover craft!!
			#if stable: recovered = True
			pass
		
		# navigate to target
		navigation_phase = True
		while navigation_phase:
			# set succsessive waypoints leading to target or just target waypoint??
			mov.set_waypoint(target_pos, landing_start_alt+1000)
			if sen.current_altitude < landing_start_alt:
				#if distance_to_target < landing_start_dist:
					navigation_phase = False

		mov.start_landing()
		# test for landed
		landed = True
		
		print("mission complete")
		armed = False
	
#
# Autostart
# ------------------------------------------------------
#

def autostart():
	print("Hello ARLISS")
	mission = Mission()
	mission.main_run
	
autostart()