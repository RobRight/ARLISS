
'''

UNR ARLISS Quadrotor Mission Code
Version 0.3

written by: William Gregory
repo: https://github.com/RobRight/ARLISS/

'''

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

# Sensors Class
# ------------------------------------------------------
# call get_data() to update sensor readings
#
class Sensors:
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
	# info: under 'Python Variable: cs'
	# http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html
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


# Move class
# ------------------------------------------------------
# waypoints, land, arm/disarm
# note: need loiter function; check channel mappings
class Move:
    sen = Sensors()
	log = Logging()

	# - settings begin -
    verbose = True
    waypoint_tolerance = 2 # distance from waypoint allowed before moving on. units  ???
	default_takeoff_alt = 20 # m
	default_takeoff_speed = 2 # m/s
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
	# - settings end -

    # State variables:
    armed = False

    def __init__(self):
        pass

    # - rc begin -
	# call to engage motor at speed mapped from input of 0 to 1.
    # input:
    #   - channel (ex: rc_throttle)
    #   - double (must be in 0:1 range)
    # pass: NA
	def rc_set_value(chan, in_val):
        # check input
		if in_val > 1 or in_val < 0:
			print("error: rc_set_value() value input out of range.")
			return False
		# map value
        min_val = chan[1]
		max_val = chan[2]
        val = (max_val - min_val)*val + min_val
        # set value to channel
		Script.SendRC(chan[0], val, True)
		return True

	# set value to all four rc channels
    # input: double x4 (0:1); to skip a channel input (-1) for its value
    # pass: NA
	def rc_set_all(in_t, in_p, in_r, in_y):
		if in_t != -1: self.rc_set_value(self.rc_throttle, in_t)
		if in_p != -1: self.rc_set_value(self.rc_pitch, in_p)
		if in_r != -1: self.rc_set_value(self.rc_roll, in_r)
		if in_y != -1: self.rc_set_value(self.rc_yaw, in_y)
        return True

	# reset all rc value inputs to idle position
    # pass: NA
	def rc_reset_all():
		# throttle min, pitch half, roll half, yaw half
		self.rc_set_all(0.0, 0.5, 0.5, 0.5)
        return True

	# arm craft.  failsafes must pass.
	# pass: NA
	def arm_craft(self):
		if (self.verbose) print("arm_craft() - arming motors")
		self.change_mode_loiter()
		self.rc_reset_all()
		self.rc_set_all(0,-1,-1,1) # left stick to bottom right.  throttle and yaw channels
		Script.WaitFor('ARMING MOTORS',15000)
		self.rc_reset_all()
		if (self.verbose) print("arm_craft() - motors armed")
		self.armed = True
		return True

	# disarm motors (low power mode)
	# pass: NA
	def disarm_craft(self):
		if (self.verbose) print("disarm_craft() - disarming motors")
		self.change_mode_loiter()
		self.rc_reset_all()
		self.rc_set_all(0,-1,-1,0) # left stick bottom left. throttle and yaw channels
		Script.WaitFor('DISARMING MOTORS',15000)
		self.rc_reset_all()
		if (self.verbose) print("disarm_craft() - motors disarmed")
		self.armed = False
		return True
    # - rc end -

	# - waypoints begin -
	# passing all (loc,lng,alt) values will send this waypoint to craft, begining immediatly.
    # input: (int) latitude, (int) longitude, (int) altitude (units???)
    # pass: NA
    def set_waypoint(self, loc_lat, loc_lng, loc_alt):
    #http://www.diydrones.com/forum/topics/mission-planner-python-script?commentId=705844%3AComment%3A1306487
    new_wp = MissionPlanner.Utilities.Locationwp() # create waypoint object
    MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,loc_lat) # set waypoint latitude
    MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,loc_lng) # set waypoint longitude
    MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,loc_alt) # set waypoint altitude
    MAV.setGuidedModeWP(new_wp) # begin waypoint
    # prints waypoint info.
    if (self.verbose) print("waypoint set: lat:" + str(loc[0]) + " lng:" + str(loc[1]) + " alt:" + str(loc[2]))

	# returns True if distance to current waypoint minimum value 'waypoint_tolerance' is met
    # pass: NA
    def waypoint_complete(self):
        if self.sen.current_distance < self.waypoint_tolerance: return True
        return False

	# waits for waypoint_complete() to finish
    # note: blocking functions.  could add timeout
    # pass: NA
    def wait_waypoint_complete(self):
        while(self.waypoint_complete()): pass
        if (self.verbose) print("wait_waypoint_complete() - waypoint complete")
	# - waypoints end -

    # - modes begin -
    # engage landing
	# pass: NA
	def change_mode_landing(self):
		# http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_land
		# MAV_CMD_NAV_LAND(0,0,0,0,l,l) - NODE: do not know if this command works
		Script.ChangeMode('LAND') # set mode to LAND - http://copter.ardupilot.com/wiki/land-mode/
		# test for landed
		Script.WaitFor('LAND', 5000)

    # change mode to loiter
	# info: hold position and altitude
	# pass: NA
	def change_mode_loiter(self):
		Script.ChangeMode('LOITER')
		Script.WaitFor('LOITER', 5000) # whats this do?

	# return to launch site and land
	# pass: NA
	def change_mode_rtl(self):
		Script.ChangeMode('RTL')
		Script.WaitFor('RTL', 5000)

	# takeoff for testing and non-assisted flying
	# pass: NA - not ready
    '''
	def change_mode_takeoff(self):
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
			#''
			# monitor vertical speed
			if current_verticle_speed < takeoff_speed-2:
				throttle_val + 0.01
			if current_verticle_speed > takeof_speed+2:
				throttle_val - 0.01
			#''
			# update altitude
			temp_current_alt = self.sen.current_altitude
    '''
    # - modes end -

	# setup motor code stuff
	def setup(self):
        # inital check before script will accept control of craft.
        # check failsafes
        # check disarmed
		self.rc_reset_all()
        self.change_mode_loiter()

#
# Mission Class
# ------------------------------------------------------
# setup for tests of minor classes (Loggin, Sensors, Move)
class Mission:
    mov = Move()
	sen = Sensors()
	log = Logging()

    # - settings begin -
    # mission options:
	# ----------------
	# (0) no mission
    # (1) test sensors - displays current sensor data
    # (2) test arm and disarm - arms on ground, seconds later disarms.
	# (3) test takeoff and landing - goes up and lands at the same location.
	# (4) test waypoints - flys to a few waypoints, then rtl and land.  note: check waypoints locations and rtl land settings in MP.
    #
	# () test recovery (NI) - not implemented (This needs to be written and tested, possibly some research into decent at hight speed and needs to start in free-fall.)
	# - - - - - - - -
	# (6) - mission_alpah - complete mission from idle, launch, recovery, navigation, and landling.
	# -----------------
	mission_mode = 0
    # -----------------
    verbose = True
	log_data = True
    # - settings end -

    # state variables
	launch_time = 0
	launch_pos = [0.0, 0.0]
	launch_alt = 0
    landed_time = 0
	landed_pos = [0.0, 0.0]
	landed = False
	mission_complete = False

    def __init__(self):
        pass

    def setup():
        pass

    def autorun():
        pass
