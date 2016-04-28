#!/usr/bin/python

'''
ARLISS Quadrotor Mission Code
Version 0.2

repo: https://github.com/RobRight/ARLISS/

Todo:
	test modes 1-3
	 - landing
	 - waypoints
	 - test sensors
	test RC control
		- finish arm/disarm and takeoff

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
	directory = os.path.expanduser('~') + '\Desktop\CODE_LOGS'
	start_time = ();

	# set directory and create folder if not found
	def __init__(self):
		# http://askubuntu.com/questions/138922/path-to-user-desktop-using-python
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
	log = Logging()

	# --- SETTINGS START ---
	default_takeoff_alt = 20 # m
	default_takeoff_speed = 2 # m/s

	# RC input [pin, min, max]
	rc_pin_throttle = [3, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
	rc_pin_pitch = [2, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')] # BACKWARDS
	rc_pin_roll = [1, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX')]
	rc_pin_yaw = [4, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX')]
	# --- SETTINGS END ---

	def __init__(self):
		# craft should be disarmed
		# change_mode_loiter()
		# zero out RC throttle
		# Script.SendRC(rc_throttle, 1000, True)
		pass

	# engage craft for flight
	# TESTED: False
	def arm_craft(self):
		'''
		print("disarming motors")
		change_mode_loiter()
		Script.SendRC(3, 1000, True)
		Script.SendRC(4, 1000, True)
		Script.WaitFor('DISARMING MOTORS',15000)
		Script.SendRC(4, 1500, True)
		print("MOTORS DISARMED")
		return True
		'''
		pass

	# disarm motors (low power mode)
	# TESTED: False
	def disarm_craft(self):
		'''
		print("disarming motors")
		change_mode_loiter()
		Script.SendRC(3, 1000, True)
		Script.SendRC(4, 1000, True)
		Script.WaitFor('DISARMING MOTORS',15000)
		Script.SendRC(4, 1500, True)
		print("MOTORS DISARMED")
		return True
		'''
		pass

	def test_rc(self):
		pin = rc_pin_throttle[0]
		min_val = rc_pin_throttle[1]
		max_val = rc_pin_throttle[2]
		Script.SendRC(pin, max_val/2, True)
		time.sleep(2)
		Script.SendRC(pin, min_val, True)

	# set new waypoint with altitude
	# def set_waypoint(wp_lat, wp_lng, wp_alt):
	# TESTED: False
	def set_waypoint(self,loc):
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
		'''
		takeoff_alt = default_takeoff_alt
		takeoff_speed = default_takeoff_speed
		# must do manually?
		print("begin takeoff procedure")
		temp_start_alt = sen.current_altitude
		# arm
		arm_craft()
		# enter loiter mode
		mov.change_mode_loiter()
		# engage motors
		Script.SendRC(rc_pin_throttle, 1000, True)
		# monitor vertical speed
		if current_verticle_speed < takeoff_speed-2:
			thottle+1
		if current_verticle_speed > takeof_speed+2:
			thottle-1
		# maintain until at set altitude
		if current_altitude > sen.temp_start_alt + takeoff_alt:
			pass
		'''

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
		"- wind_speed: " + str(current_wind_speed) + "\n" + "\n"
		return sen_data

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
	# () test_recovery	note: note implemented (This needs to be written and tested, posibly some resesarch into decent at hight spe)
	# - - - - - - - -
	# () - mission_alpah -
	# -----------------
	mission_mode = 0

	requrie_gps_lock = True

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
	armed = False # set to True to engage system
	mission_text = ''

	# --- saved locations ---
	save_loc_home = [39.4158266, -119.7347143, 60] # by cars
	save_loc_0 = [39.4164607, 119.7364712, 60] # middle of field
	save_loc_1 = [39.4158556, -119.7354412, 60] # middle, closer to cars

	def __init__(self):
		self.log.log_enable = self.log_data
		start_time = time.localtime()
		self.log.start_time = start_time

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

	# Test -------------------
	# take off from ground, reach set altitude and loiter
	def test_takeoff(self):
		mov.takeoff_from_ground()

	# land at given position
	def test_landing(self):
		# start from air
		mov.set_waypoint(save_loc_home)
		mov.change_mode_landing()

	# test waypoint navigation
	def test_waypoint(self, op):
		if op:
			# ask user for position with altitude
			temp_wp_lat = input("Enter waypoint latitude")
			temp_wp_lon = input("Enter waypoint longitude")
			temp_wp_alt = input("Enter waypoint altitude")
			temp_wp = [temp_wp_lat, temp_wp_lon, temp_wp_alt]
			mov.set_waypoint(temp_wp)
		else:
			# use known posistion
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
			# loop twise a sencond
			time.sleep(1)

	# Main ------------------
	# recover craft from unstable flight
	def phase_recover_craft(self):
		pass

	# fly to waypoint
	def phase_directed_flight(self):
		mov.set_waypoint(save_loc_0)

	# -------------------------
	# ------------
	# mission alpha code start
	# ------------
	def mission_alpha(self):
		start()

		wait_for_rocket_launch()
		wait_for_rocket_ejected()

		# wait until recovery start alt
		wait_for_altitude(recovery_start_alt, True)

		# begin recovery
		recover_craft()

		# navigate to target
		navigation_phase = True
		while navigation_phase:
			# set succsessive waypoints leading to target or just target waypoint??
			mov.set_waypoint(target_pos, landing_start_alt+1000)
			# reached the target?
			if check_within_landing() == True:
				navigation_phase = False

		mov.start_landing()
		landed = True

		print("mission complete")
		armed = False
	# ------------
	# mission alpha code end
	# ------------

	# main mission code runner
	def run_main(self):
		while self.mission_complete == False:
			if self.debug: print("starting: run_main")
			# MAIN RUN CODE HERE
			# depends on current mission
			# (0) no mission
			if self.mission_mode == 0:
				self.mission_text = 'no mission'
			# (1) test_landing
			elif self.mission_mode == 1:
				self.mission_text = 'test_landing'
				self.test_landing()
			# (2) test_navigation
			elif self.mission_mode == 2:
				self.mission_text = 'test_navigation'
				self.test_waypoint(False)
			# (3) test_sensors
			elif self.mission_mode == 3:
				self.mission_text = 'test_sensors'
				self.loop_sensor_output()
			# (11) - mission_alpha -
			elif self.mission_mode == 11:
				self.mission_text = 'mission_alpha'
				#self.mission_alpha()
			else:
				print("error: mission_mode unknown")
			# mission compelte
			self.mission_complete = True

	# call at mission start
	def run_startup(self):
		while self.mission_complete == False:
			if self.debug: print("starting: run_startup")
			self.print_intro()
			#time.sleep(5)
			self.warm_up()
			self.setup()
			self.print_start()
			self.run_main()

# autostart
def autostart():
	print("online")
	mission = Mission()
	mission.run_startup()
	print("mission complete")

autostart()
