#!/usr/bin/python

'''
ARLISS Quadrotor Mission Code
Version 0.2

repo: https://github.com/RobRight/ARLISS/



'''

#import sys
#import math
import os
import time
import clr
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink") # includes the Utilities class
import MAVLink

#
# Logging Class
# ------------------------------------------------------
#
class Logging:
	log_enable = True
	directory = '' # path to user directory
	start_time = (); # assigned at start by 'Mission' class

	# set directory and create folder if not found
	def __init__(self):
		# http://askubuntu.com/questions/138922/path-to-user-desktop-using-python
		directory = os.path.expanduser('~') + '\Documents\CODE_LOGS'
		# http://stackoverflow.com/questions/273192/how-to-check-if-a-directory-exists-and-create-it-if-necessary
		if not os.path.exists(self.directory):
			os.makedirs(self.directory)

	# generate a file name directory and time stanp
	def generate_filename(self, given):
		filename = given + "_" + time.strftime("%H_%M_%S", self.start_time)
		filename2 = os.path.join(self.directory, filename + ".txt")
		return filename2

	# clear given file
	def clear_log(self, in_f_name):
		if self.log_enable:
			filename = self.generate_filename(in_f_name)
			f = open(filename, 'w')
			f.close()

	# write given data to file
	def log_data(self, in_data, in_f_name):
		if self.log_enable:
			filename = self.generate_filename(in_f_name)
			f = open(filename, 'a')
			f.write(in_data)
			f.close()

#
# Move Class
# ------------------------------------------------------
#
class Move:
	sen = Sensors()
	log = Logging()

	# --- SETTINGS START ---
	default_takeoff_alt = 20 # m
	default_takeoff_speed = 2 # m/s

	esc_min = 0
	esc_max = 0

	# RC input [pin, min, max]
	rc_throttle = [3, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
	rc_pitch = [2, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')] # BACKWARDS
	rc_roll = [1, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX')]
	rc_yaw = [4, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX')]

	# ESC output [pin, min, max]
	#esc_fr = [1, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
	#esc_br = [2, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
	#esc_bl = [3, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
	#esc_fl = [4, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
	# --- SETTINGS END ---

	armed = False # set to True to engage system

	def __init__(self):
		# craft should be disarmed
		# change_mode_loiter()
		# zero out RC throttle
		# Script.SendRC(rc_throttle, 1000, True)
		pass

	def rc_value_map(chan, val): # val: 0 to 1
		min_val = chan[1]
		max_val = chan[2]
		return (max_val - min_val)*val + min_val

	# call to engage motor at 0 to 1 speed
	def rc_set_value(chan, in_val):
		if in_val > 1:
			print("error: rc_set_value greater than one.")
			return False
		val = rc_value_map(chan, in_val)
		Script.SendRC(chan[0], val, True)
		return True

	# set value to all four rc channels
	def rc_set_all(in_t, in_p, in_r, in_y):
		if in_t != -1: rc_set_value(rc_throttle, in_t)
		if in_p != -1: rc_set_value(rc_pitch, in_p)
		if in_r != -1: rc_set_value(rc_roll, in_r)
		if in_y != -1: rc_set_value(rc_yaw, in_y)

	# reset all rc value inputs to idle
	def rc_reset_all():
		# throttle min, pitch half, roll half, yaw half
		rc_set_all(rc_value_map(rc_throttle, 0), \
			rc_value_map(rc_pitch, 0.5), \
			rc_value_map(rc_roll, 0.5), \
			rc_value_map(rc_yaw, 0.5))

	# engage craft for flight
	# TESTED: False
	def arm_craft(self):
		print("arming motors")
		change_mode_loiter()
		rc_reset_all()
		rc_set_all(0,0,1,1)
		Script.WaitFor('ARMING MOTORS',15000)
		rc_reset_all()
		print("MOTORS ARMED")
		self.armed = True
		return True

	# disarm motors (low power mode)
	# TESTED: False
	def disarm_craft(self):
		print("disarming motors")
		change_mode_loiter()
		rc_reset_all()
		rc_set_all(0,0,0,0)
		Script.WaitFor('DISARMING MOTORS',15000)
		rc_reset_all()
		print("MOTORS DISARMED")
		self.armed = False
		return True

	# test of rc control
	def test_rc(self, test):
		if test == 0:
			pin = rc_throttle[0]
			min_val = rc_throttle[1]
			max_val = rc_throttle[2]
			Script.SendRC(pin, max_val/2, True)
			time.sleep(2)
			Script.SendRC(pin, min_val, True)r
		elif test == 1:
			# wave
			# one at a time
			val = 0.15
			rc_set_value(esc_fr, val)
			time.sleep(1)
			rc_set_value(esc_br, val)
			time.sleep(1)
			rc_set_value(esc_bl, val)
			time.sleep(1)
			rc_set_value(esc_fl, val)
			time.sleep(1)
			# all at once
			pass

	# set new waypoint with altitude
	# def set_waypoint(wp_lat, wp_lng, wp_alt):
	# TESTED: False
	def set_waypoint(self,loc): # add a wait for waypoint complete??
		#http://www.diydrones.com/forum/topics/mission-planner-python-script?commentId=705844%3AComment%3A1306487
		new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
		MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,loc[0])		# set waypoint latitude
		MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,loc[1])		# set waypoint longitude
		MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,loc[2])		# set waypoint altitude
		MAV.setGuidedModeWP(new_wp)											# fly to the new waypoint

	# engage landing
	# TESTED: False
	def change_mode_landing(self):
		# http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_land
		# MAV_CMD_NAV_LAND(0,0,0,0,l,l) - NODE: do not know if this command works
		Script.ChangeMode('LAND') # set mode to LAND - http://copter.ardupilot.com/wiki/land-mode/
		# test for landed
		Script.WaitFor('LAND', 5000)

	# hold position and altitude
	# TESTED: False
	def change_mode_loiter(self):
		Script.ChangeMode('LOITER')
		Script.WaitFor('LOITER', 5000) # whats this do?

	# return to launch site and land
	# TESTED: False
	def change_mode_rtl(self):
		Script.ChangeMode('RTL')
		Script.WaitFor('RTL', 5000)

	# takeoff for testing and non-assisted flying
	# TESTED: False
	def takeoff_from_ground(self):
		takeoff_alt = default_takeoff_alt
		takeoff_speed = default_takeoff_speed
		# must do manually?
		print("begin takeoff procedure")
		temp_start_alt = self.sen.current_altitude
		# arm
		self.arm_craft()
		# enter loiter mode
		self.change_mode_loiter()
		# engage motors; warmup then throttle at 80%
		self.rc_reset_all()
		self.rc_set_all(0.15,-1,-1,-1)
		time.sleep(1)

		throttle_val = 0.8
		temp_current_alt = self.sen.current_altitude
		# maintain until at set altitude
		while temp_current_alt < temp_start_alt + takeoff_alt:
			# set throttle
			#self.rc_set_all(throttle,-1,-1,-1)
			self.rc_set_value(rc_throttle, throttle_val)s
			'''
			# monitor vertical speed
			if current_verticle_speed < takeoff_speed-2:
				throttle_val + 0.01
			if current_verticle_speed > takeof_speed+2:
				throttle_val - 0.01
			'''
			# update altitude
			temp_current_alt = self.sen.current_altitude

	# setup motor code stuff
	def setup(self):
		rc_reset_all()

#
# Sensors Class
# ------------------------------------------------------
#
class Sensors:
	log = Logging()
	# state variables
	current_time = 0
	current_mode = ''
	current_distance = 0
	current_waypoint = 0
	current_altitude = 0.0
	current_vertical_speed = 0.0
	current_ground_speed = 0.0
	current_ground_course = 0.0
	current_wind_direction = 0.0
	current_wind_speed = 0.0
	current_roll = 0.0
	current_pitch = 0.0
	current_yaw = 0.0
	current_lat = 0.0
	current_lng = 0.0
	current_gps_stat = 0.0
	current_gps_count = 0.0
	current_battery_voltage = 0.0
	current_armed = False
	current_altitude_error = 0.0
	current_accel = []
	current_gyro = []

	# init
	def __init__(self):
		pass

	# get current flight data
	def get_data(self):
		#print("got new sensor data")
		global current_time, current_mode, current_distance, current_altitude, \
			current_vertical_speed, current_ground_speed, current_ground_course, \
			current_waypoint, current_wind_direction, current_wind_speed, current_roll, \
			current_pitch, current_yaw, current_lat, current_lng, current_gps_stat, \
			current_gps_count, current_battery_voltage, current_armed, \
			current_accel, current_gyro
		current_time = time.time()
		current_mode = cs.mode
		current_distance = cs.wp_dist
		current_waypoint = cs.wpno
		current_altitude = cs.alt
		current_vertical_speed = cs.verticalspeed
		current_ground_speed = cs.groundspeed
		current_ground_course = cs.groundcourse
		current_wind_direction = cs.wind_dir
		current_wind_speed = cs.wind_vel
		current_roll = cs.roll
		current_pitch = cs.pitch
		current_yaw = cs.yaw
		current_lat = cs.lat
		current_lng = cs.lng
		current_gps_stat = cs.gpsstatus
		current_gps_count = cs.satcount
		current_battery_voltage = cs.battery_voltage
		current_armed = cs.armed
		current_altitude_error = cs.alt_error

		current_accel = []
		current_accel.append(cs.ax)
		current_accel.append(cs.ay)
		current_accel.append(cs.az)

		current_gyro = []
		current_gyro.append(cs.gx)
		current_gyro.append(cs.gy)
		current_gyro.append(cs.gz)

	# output current flight data
	def print_data(self):
		print("- time: " + str(current_time))
		print("- mode: " + current_mode)
		print("- wp_distance: " + str(current_distance))
		print("- wpno: " + str(current_waypoint))
		print("- altitude: " + str(current_altitude))
		print("- vertical_speed: " + str(current_vertical_speed))
		print("- ground_speed: " + str(current_ground_speed))
		print("- ground_course: " + str(current_ground_course))
		print("- wind_direction: " + str(current_wind_direction))
		print("- wind_speed: " + str(current_wind_speed))
		print("- rol: " + str(current_roll))
		print("- pitch: " + str(current_pitch))
		print("- yaw: " + str(current_yaw))
		print("- latitude: " + str(current_lat))
		print("- longitude: " + str(current_lng))
		print("- gps_stat: " + str(current_gps_stat))
		print("- gps_count: " + str(current_gps_count))
		print("- battery_voltage: " + str(current_battery_voltage))
		print("- armed: " + str(current_armed))
		print("- altitude_error: " + str(current_altitude_error))
		print("- accel.x: " + str(current_accel[0]))
		print("- accel.y: " + str(current_accel[1])))
		print("- accel.z: " + str(current_accel[2]))
		print("- gyro.x: " + str(current_gyro[0]))
		print("- gyro.y: " + str(current_gyro[1]))
		print("- gyro.z: " + str(current_gyro[2]))

	# make one string with logged data
	def return_string_of_data():
		sen_data = "sensors current:" + "\n" + \
		"- time: " + str(current_time) + "\n" + \
		"- mode: " + current_mode + "\n" + \
		"- wp_distance: " + str(current_distance) + "\n" + \
		"- wpno: " + str(current_waypoint) + "\n" + \
		"- altitude: " + str(current_altitude) + "\n" + \
		"- vertical_speed" + str(current_vertical_speed) + "\n" + \
		"- ground_speed: " + str(current_ground_speed) + "\n" + \
		"- ground_course: " + str(current_ground_course) + "\n" + \
		"- wind_direction: " + str(current_wind_direction) + "\n" + \
		"- wind_speed: " + str(current_wind_speed) + "\n" + \
		"- rol: " + str(current_roll) + "\n" + \
		"- pitch: " + str(current_pitch) + "\n" + \
		"- yaw: " + str(current_yaw) + "\n" + \
		"- latitude: " + str(current_lat) + "\n" + \
		"- longitude: " + str(current_lng) + "\n" + \
		"- gps_stat: " + str(current_gps_stat) + "\n" + \
		"- gps_count: " + str(current_gps_count) + "\n" + \
		"- battery_voltage: " + str(current_battery_voltage) + "\n" + \
		"- armed: " + str(current_armed) + "\n" + \
		"- altitude_error: " + str(current_altitude_error) + "\n" + \
		"- accel.x: " + str(current_accel[0]) + "\n" + \
		"- accel.y: " + str(current_accel[1]) + "\n" + \
		"- accel.z: " + str(current_accel[2]) + "\n" + \
		"- gyro.x: " + str(current_gyro[0]) + "\n" + \
		"- gyro.y: " + str(current_gyro[1]) + "\n" + \
		"- gyro.z: " + str(current_gyro[2]) + "\n" + "\n"
		return sen_data

	# log flight data to file
	def log_data(self):
		sen_data = self.return_string_of_data()
		# log data
		self.log.log_data(sen_data, "data")
		print("data logged")

#
# Mission Class
# ------------------------------------------------------
#
class Mission:
	mov = Move()
	sen = Sensors()
	log = Logging()

	# mission options:
	# ----------------
	# (0) no mission
	# - - - - - - - -
	# () test_takeoff	note: not implemented (unsure of command)
	# () test_landing	note: must provide landing location
	# () test_waypoint	note: must provide waypoint location
	# () test_recovery	note: note implemented (This needs to be written and tested, possibly some research into decent at hight speed)
	# - - - - - - - -
	# () - mission_alpah -
	# -----------------
	mission_mode = 0

	requrie_gps_lock = True
	testing = True

	verbose = True
	debug = True
	log_data = True

	# --- SET BEFORE MISSION ALPHA ---
	target_pos = [31.002, -110.010]
	recovery_start_alt = 1500.0
	landing_start_alt = 20.0
	landing_start_dist = 10.0
	# --- SET BEFORE MISSION ALPHA ---

	# state variables
	launch_time = 0
	launch_pos = [0.0, 0.0]
	launch_alt = 0
	rocket_launched = False # true once rocket launched
	ejected_time = 0
	ejected_pos = [0.0, 0.0]
	ejected_alt = 0
	ejected = False # true once released from rocket
	recovered_time = 0
	recovered_pos = [0.0, 0.0]
	recovered_alt = 0
	recovered = False # true once in stable flight
	landed_time = 0
	landed_pos = [0.0, 0.0]
	landed = False
	mission_complete = False
	mission_text = ''

	# --- saved locations ---
	save_loc_home = [39.4158266, -119.7347143] # by cars
	save_loc_0 = [39.4164607, 119.7364712] # middle of field
	save_loc_1 = [39.4158556, -119.7354412] # middle, closer to cars

	def __init__(self):
		self.log.log_enable = self.log_data
		start_time = time.localtime()
		self.log.start_time = start_time
		mov.setup()

	# Utils ------------------
	def print_intro(self):
		self.log.log_data("online", "log")
		print("mission online")
		print("mode: " + str(self.mission_mode) + ":" + self.mission_text)
		print("")
		print("warming up..")

	def print_start(self):
		self.log.log_data("ready", "log")
		print("warmup complete")
		print("starting mission")
		print("")

	def check_gps_lock(self):
		if self.debug: print("starting: check_gps_lock")
		if self.sen.current_gps_stat == 1:
			if self.sen.current_gps_count > 3:
				return True
		return False

	# check sensors on start
	def warm_up(self):
		if self.debug: print("starting: warm_up")
		# wait until gps lock
		if self.requrie_gps_lock == True:
			locked = False
			temp_st = time.time()
			while locked != True:
				# add timeout
				self.sen.get_data()
				locked = self.check_gps_lock()
				if self.debug: print("- done")
				time.sleep(0.1)
				if temp_st - time.time() > 10:
					print("error: check_gps_lock")
					self.mission_complete = True
			print("- GPS locked")
		else:
			print("- skipping GPS")
		print("- done")

	# start code
	def setup(self):
		if self.debug: print("starting: setup")
		flight_mode = 0
		launch_pos = [self.sen.current_lat, self.sen.current_lng]
		launch_alt = self.sen.current_altitude
		print("- done")

	# ------------------------------
	# test until rocket launched
	def wait_for_rocket_launch(self):
		while rocket_launched == false:
			if sen.current_altitude>start_alt+1000:
				rocket_launched = True
		print("rocket launched")

	# test until rocket ejected
	def wait_for_rocket_ejected(self):
		while ejected == false:
			if sen.current_altitude>max_alt:
				max_alt = sen.current_altitude
			if sen.current_altitude<max_alt+200:
				ejected = True # note: maybe use accelerometer
		print("quad ejected")
		ejected_pos = [sen.curren_lat, sen.curren_lon]

	# wait for set altitude
	def wait_for_altitude(self, des_alt, below):
		# add timeout
		ca = sen.current_altitude
		if below:
			while ca>des_alt:
				ca = sen.current_altitude
		else:
			while ca<des_alt:
				ca = sen.current_altitude
		return True

	def check_within_landing(self):
		pass

	def arm_craft():
		self.move.arm_craft()

	def disarm_craft():
		self.mov.disarm_craft()

	def land_craft():
		self.mov.start_landing()
		self.landed = True

	def set_mission_text():
		# (0) no mission
		if self.mission_mode == 0:
			self.mission_text = 'no mission'
		# (1) test_landing
		elif self.mission_mode == 1:
			self.mission_text = 'test_landing'
		# (2) test_navigation
		elif self.mission_mode == 2:
			self.mission_text = 'test_navigation'
		# (3) test_sensors
		elif self.mission_mode == 3:
			self.mission_text = 'test_sensors'
		# (11) - mission_alpha -
		elif self.mission_mode == 11:
			self.mission_text = 'mission_alpha'
		else:
			print("error: mission_mode unknown")

	# Test -------------------
	# take off from ground, reach set altitude and loiter
	def test_takeoff(self):
		mov.takeoff_from_ground()

	# land at given position
	def test_landing(self):
		# start from air
		mov.set_waypoint(save_loc_home)
		mov.change_mode_landing()

	# set waypoint given a saved position and altitiude
	def set_waypoint(self, pos[], alt):
		lat = pos[0]
		lng = pos[1]
		mov.set_waypoint([lat, lng, alt])

	#  enter custom waypoint
	def custom_waypoint(self):
		pass
	# test waypoint navigation
	def test_waypoint(self, op):
		if op:
			# ask user for position with altitude
			temp_wp_lat = input("Enter waypoint latitude")
			temp_wp_lon = input("Enter waypoint longitude")
			temp_wp_alt = input("Enter waypoint altitude")
			temp_wp = [temp_wp_lat, temp_wp_lon, temp_wp_alt]
			mov.set_waypoint(temp_wp)

	# test recovery from free-fall
	def test_recovery(self):
		self.recover_craft()

		else:
			# use known position
			mov.set_waypoint(save_loc_1)

	# test recovery from free-fall
	def test_recovery(self):
		recover_craft()

	def loop_sensor_output(self):
		while True:
			if self.debug: print("starting: loop_sensor_output")
			self.sen.get_flight_data()
			self.sen.print_data()
			self.sen.log_data()
			# loop twice a second
			time.sleep(1)
			if self.debug: print("- done")

	# Main ------------------
	# recover craft from unstable flight
	def phase_recover_craft(self):
		pass

	# fly to waypoint
	def phase_directed_flight(self):
		if self.debug: print("starting: phase_directed_flight")
		self.navigation_phase = True
		while self.navigation_phase:
			# set successive waypoints leading to target or just target waypoint??
			self.mov.set_waypoint(self.target_pos, self.landing_start_alt+1000)
			if self.check_within_landing() == True:
				self.navigation_phase = False
		if self.debug: print("- done")

	# ------------
	# mission alpha code start
	# ------------
	def mission_alpha(self):
		# wait for launch
		self.wait_for_rocket_launch()
		self.wait_for_rocket_ejected()
		# wait until recovery start alt
		self.wait_for_altitude(recovery_start_alt, True)
		# begin recovery
		self.recover_craft()
		# navigate to target
		self.phase_directed_flight()
		# land then disarm
		self.land_craft()
		self.disarm_craft()
	# ------------
	# mission alpha code end
	# ------------

	# main mission code runner
	def run_main(self):
		self.start()
		while self.mission_complete == False:
			if self.debug: print("starting: run_main")
			# (0) no mission
			if self.mission_mode == 0:
				pass
			# (1) test_landing
			elif self.mission_mode == 1:
				self.test_landing()
			# (2) test_navigation
			elif self.mission_mode == 2:
				self.test_waypoint(False)
			# (3) test_sensors
			elif self.mission_mode == 3:
				self.loop_sensor_output()
			# (11) - mission_alpha -
			elif self.mission_mode == 11:
				self.mission_alpha()
			else:
				print("error: mission_mode unknown")
			self.mission_complete = True
			if self.debug: print("- done")

	# call at mission start
	def run_startup(self):
		while self.mission_complete == False:
			if self.debug: print("starting: run_startup")
			self.print_intro()
			time.sleep(5)
			self.warm_up()
			self.setup()
			self.print_start()
			self.run_main()

# autostart
def autostart():
	run_test_code = False
	if (run_test_code):
		# test code

		# test code
	else:
		print("online")
		mission = Mission()
		mission.run_startup()
		print("mission complete")

autostart()
