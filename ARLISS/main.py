#
# UNR ARLISS 2016 Mission Code
# Autonomous navigation and control of quad-rotor using Pixhawk flight controller
# Date: 7/1/2016 to 09/1/2016
#
# 
# Copyright (c) 2016 William Gregory
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
# -----------------------------------------------------------------------------



# GitHub: https://github.com/robright/arliss
# MP command info: http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html
#
# Using Python 2.7 and Mission Planner by ArduPilot
#
#
# Mission Setup Notes:
# - craft should be armed and porps not spinning before being put into the can.
#
#
# ToDo:
# - test parameter manage function (GPS, Fail-safes)
# -* complete parameter manage for PIDs (separate PIDs for recovery and navigation?)
# - add battery monitor function w/ periodic update
# -* add periodic status report. distance to target.  distance covered.  own log file?
# - add periodic checks that craft is stable and on track (maybe)
# -* add check for GPS status and GPS dependent modes (warning when GPS required and disabled)
# - discuss adding sonar to the craft for landing
# - test altitude reading when GPS is disabled using barometer (same sensor variable?)
# - look into Python memory management
# - complete Rocket class functions
# - prefix Config variable names with the function they are used with
# -* test Pixhawk sensor variable update rate
# - add periodic checks before payload release that launch or payload release was not missed due to telemetry loss
# - add threads (ex: waiting_for_release) (maybe)
# -* check location setting works properly
# - check test_movement_speeds() test three currently disabled
# - consider not moving straight down for recovery to avoid prop wash
# -* add switch to craft in contact with payload can to detect release (wait_for_payload_release)
# -* test altitude reading using barometer while GPS is disabled
#
# Done:
# - redo Sensor class for speed
# - add test_movement function for max ground speed and vertical speed
# - move default configuration functions to Config class
# - add detect launch function
# - add detect payload release function
# - add timeout on wait_for_recover and retry after a delay
# - disable GPS before launch; get fix once payload released

import math  # math
import os  # log file path
import time  # sleep and current time
import clr  # AddReference
clr.AddReference("MissionPlanner")
clr.AddReference("MissionPlanner.Utilities")
clr.AddReference("MAVLink")  # includes the Utilities class
import MissionPlanner
from MissionPlanner.Utilities import Locationwp
import MAVLink  # needed?
#from thread import start_new_thread

#
# ------------------------------------------------------
# preset options:
# -----------------------
# ("NA")     do not use preset
# ("ma-01")  run ma-01 mission with default parameters
# ("dam_t4")
# ("dam_t5") run t5 mission with default parameters
# ("dam_t6") run t6 mission with default parameters

# mission options:
# -----------------
# (0) no mission
# (t1) test arm and disarm - arms on ground, seconds later disarms.
# (t2) test takeoff and landing - goes up and lands at the same location.
# (t3) test way-points - flies to a few way-points, then RTL and land
# (t4) test recovery - check settings in Config class
# (t5) test_navigation
# (t6) test_movement speeds
# (ma-01) - mission_alpah - complete mission from idle, launch, recovery, navigation, and landing.

# location options:
# -----------------
# (dam) Damonte Ranch
# (brd) Black Rock Desert
# ------------------------------------------------------

#
# Config Class
# ------------------------------------------------------
# Manages script configuration for easy setting control
# general settings, test settings, locations and FC pins
#
class Config:
    # --------------------------
    # - preset option -
    # use this option to auto configure the Config class settings.
    # if you use this option, most settings will be over written at runtime.
    # if you do not use this option, set it to "NA".
    preset_config = "dam_t6"  # options ("NA", "dam_t5", dam_t6)
    # --------------------------
    # - general settings -
    mission_mode = "0"  # MC - specific mission to run.  see options above. (0)
    location = "dam"  # flying location.  default: dam (Damonte Ranch)
    run_test = False # sensor and file testing. (False)
    require_disarm = False  # check at start to require disarm.  False is starting in air. (False)
    disable_gps_on_start = False  # True to disable GPS on code start.  (False)
    battery_update_interval = 2  # difference in percentage to update
    # navigation
    jump_distance = 100  # MC - distance to jump each time. - needs testing. (100)
    jump_alt = 80  # MC - vertical distance to jump. - needs testing. (80)
    # payload release detection
    pr_use_alt = True  # use altitude trigger (True) or switch trigger (False)
    pr_alt_sleep_test = 1  # time to sleep before re-sampling altitude
    pr_alt_trigger = 5  # difference in altitude over 'pr_alt_sleep_test' second(s) to detect payload release
    # recovery
    wait_recov = True  # wait until recovered (True)
    wr_vs_max = 2.0  # wait for recover, vertical speed below this value. (1.0) (good?)
    wr_timeout_time = 10  # time before timeout during wait_for_recover in seconds (10) (good?)
    wr_retry_time = 2  # time before retry after recover in seconds (2)
    # takeoff
    takeoff_throttle_val = 0.7  # starting throttle value for takeoff. (70%)
    default_takeoff_alt = 20  # distance in meters from starting location to reach in takeoff. (20 m)
    default_takeoff_speed = 2  # vertical speed goal during takeoff in meters per second. (2 m/s)
    # land
    desired_vert_speed = -0.2 # MC - once vertical speed drops below this value, assume landed. (-0.2 m/s)
    # way-points
    waypoint_tolerance = 10  # MC - distance away from way-points in meters to consider it completed. (5 m)
    # launch detection
    launch_trigger_altitude = 100 # altitude in meters from ground to acknowledge launch (100 m)
    # ---------------------------
    # - testing class -
    # test_arm function
    test_disarm = False  # not implemented - include disarm after arm (False)
    # test_waypoints function
    include_takeoff_wp = True  # after takeoff, set current position as a way-point to maintain control. (True)
    testing_altitude = 20  # relative altitude to test way-points in meters. (20 m)
    wp_1_index = 2  # first way-point index to fly to. (2)
    wp_2_index = 5  # second way-point index to fly to. (5)
    return_after = True  # RTL after way-point tests. (True)
    # test takeoff and landing
    include_takeoff_t = True  # include takeoff in test.  otherwise, start in the air. (True)
    hold_position_time = 10  # time to hold in the air before landing in seconds. (10 s)
    # test recovery
    test_recover_start_alt = 120  # altitude to start recovery test at in meters. (120 m)
    takeoff_before_recover = True  # include takeoff in recover test. False start from air. (True)
    flyto_recover = True  # fly to starting position for recovery test (True)
    recover_test_sleep = 3  # sleep time to allow free fall in seconds (3 s)
    fly_back_home = True  # RTL after recovery test (True)
    # - logging class -
    log_enable = True  # MC - enable file logging (True)
    log_print_enable = True  # enable console logging (True)
    log_default_name = "log_file"  # file log filename prefix ("log_file")
    # - move class -
    # min distance from way-point before moving on
    # RC pins
    rc_throttle_pin = 3  # MC - (3) throttle pin
    rc_pitch_pin = 2  # MC - (2) pitch pin
    rc_roll_pin = 1  # MC - (1) roll pin
    rc_yaw_pin = 4  # MC - (4) yaw pin
    # ESC pins
    esc_f_pin = 3  # (3) - not implemented
    esc_b_pin = 4  # (4) - not implemented
    esc_l_pin = 2  # (2) - not implemented
    esc_r_pin = 1  # (1) - not implemented
    # ---------------------------
    # locations: [latitude, longitude]
    # black rock desert
    loc_brd_target = [0.0, 0.0]
    # random
    loc_rand_unr = [39.550409, -119.809378]  # UNR
    loc_rand_tahoe = [39.221711, -119.928340]  # Lake Tahoe
    # Damonte Ranch
    loc_dam = [
        [39.415847, -119.734851],  # 0
        [39.417526, -119.734867],  # 1
        [39.416714, -119.735511],  # 2
        [39.416747, -119.737741],  # 3
        [39.417728, -119.736461],  # 4
        [39.416725, -119.736518],  # 5
        [39.415816, -119.736523],  # 6
        [39.414974, -119.736552],  # 7
        [39.413872, -119.736562],  # 8
        [39.412644, -119.736585],  # 9
        [39.413439, -119.735824]]  # 10

    def config_default_t4(self):
        mission_mode = "t4"  # MC - specific mission to run.  see options above. (0)
        location = "dam"  # flying location.  default: dam (Damonte Ranch)
        run_test = False # sensor and file testing. (False)
        require_disarm = False  # check at start to require disarm.  False is starting in air. (False)
        disable_gps_on_start = False  # True to disable GPS on code start.  (False)
        battery_update_interval = 2  # difference in percentage to update
        # recovery
        wait_recov = True  # wait until recovered (True)
        wr_vs_max = 2.0  # wait for recover, vertical speed below this value. (1.0) (good?)
        wr_timeout_time = 4  # time before timeout during wait_for_recover in seconds (10) (good?)
        wr_retry_time = 2  # time before retry after recover in seconds (2)
        # test recovery
        test_recover_start_alt = 120  # altitude to start recovery test at in meters. (120 m)
        takeoff_before_recover = False  # include takeoff in recover test. False start from air. (True)
        flyto_recover = True  # fly to starting position for recovery test (True)
        recover_test_sleep = 3  # sleep time to allow free fall in seconds (3 s)
        fly_back_home = True  # RTL after recovery test (True)

    # setup configuration for navigation test
    # Mission critical: no
    # Tested: PASS-SITL-08/18
    def config_default_t5(self):
        Config.mission_mode = "t5"
        Config.location = "dam"
        Config.run_test = False
        Config.require_disarm = False
        Config.disable_gps_on_start = False
        Config.jump_distance = 100
        Config.jump_alt = 80
        Config.waypoint_tolerance = 10

    # default config for t6 (test_movement)
    def config_default_t6(self):
        Config.mission_mode = "t6"
        Config.location = "dam"
        Config.run_test = False
        Config.require_disarm = False
        Config.disable_gps_on_start = False
        Config.waypoint_tolerance = 3

    # setup at start of mission alpha
    #
    # Mission critical: yes
    # Tested: no
    def config_default_ma01(self):
    # (config and params)
        Config.mission_mode = "ma-01"
        Config.location = "brd"
        Config.require_disarm = True
        Config.disable_gps_on_start = True
        Config.jump_distance = 100
        Config.jump_alt = 80
        Config.recover_arm = False
        Config.wait_recov = True
        Config.desired_vert_speed = -0.2
        Config.waypoint_tolerance = 5
        Config.launch_trigger_altitude = 100
        Config.log_enable = True
        Config.log_print_enable = True
        Config.log_default_name = "main_log_file"
        Config.rc_throttle_pin = 3
        Config.rc_pitch_pin = 2
        Config.rc_roll_pin = 1
        Config.rc_yaw_pin = 4


# state variables
class State:
    # [start, launch, eject, recover, land]
    start_time = [0, 0, 0, 0, 0]
    start_pos = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    start_alt = [0, 0, 0, 0, 0]

    mission_begin = False
    rocket_launched = False
    rocket_payload_released = False
    landed = False
    mission_complete = False

    gps_enabled = False
    last_battery = -1


#
# Logging Class
# ------------------------------------------------------
# Manages console and file logging
# Generally only call log_data("message")
#
class Logging:
    con = Config()
    directory = ''  # path to user directory
    start_time = ()  # assigned at start by 'Mission' class

    # set directory and create folder if not found
    # Tested: PASS-SITL-07/26
    def __init__(self):
        # script start time. separate from mission start time in State class
        self.start_time = time.localtime()

        self.directory = os.path.expanduser('~') + "\Documents\CODE_LOGS"
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    # generate a file name with time and directory
    # Tested: PASS-SITL-07/26
    def generate_filename(self, given):
        filename = given + "_" + time.strftime("%m_%d_%y__%H_%M_%S", self.start_time)
        filename2 = os.path.join(self.directory, filename + ".txt")
        return filename2

    # clear given file
    # Tested: no
    def clear_log(self, in_f_name):
        if self.con.log_enable:
            filename = self.generate_filename(in_f_name)
            f = open(filename, 'w')
            f.close()

    # write given data to file
    # Tested: PASS-SITL-07/26
    def log_data(self, in_data):
        if self.con.log_enable:
            filename = self.generate_filename(self.con.log_default_name)
            f = open(filename, 'a')
            current_time = time.strftime("%H_%M_%S: ", time.localtime())
            f.write(current_time + in_data + "\n")
            f.close()
        if self.con.log_print_enable is True:
            print(in_data);

    # write given data to file with given name
    # Tested: no
    def log_data_custom(self, in_name, in_data):
        if self.con.log_enable:
            filename = self.generate_filename(in_name)
            f = open(filename, 'a')
            f.write(in_data + "\n")
            f.close()
        if self.com.log_print_enable is True:
            print(in_name + ": " + in_data)


#
# Sensors Class
# ------------------------------------------------------
# Manages the crafts flight variables.
# Call return_sensor to get most recent value
# PASS-SITL-08/18
class Sensors:
    log = Logging()

    def __init__(self):
        pass

    # # self.log.log_data("logging class - got new sensor data")

    # time.time()  # computer time () *
    # cs.mode  # flight mode (mode)
    # cs.wp_dist  # remaining way-point distance (m)
    # cs.wpno  # way-point number (#)
    # cs.alt  # altitude (m)
    # cs.verticalspeed  # vertical speed (m/s) *
    # cs.groundspeed  # ground speed (m/s)
    # cs.groundcourse  # ground course (deg)
    # cs.wind_dir  # wind direction (deg)
    # cs.wind_vel  # wind velocity (m/s)
    # cs.roll  # roll (deg) *
    # cs.pitch  # pitch (deg) *
    # cs.yaw  # yaw (deg)
    # cs.lat  # latitude (decimal degrees) *
    # cs.lng  # longitude (decimal degrees) *
    # cs.gpsstatus  # GPS status (not sure.. 3 for 3D fix?) *
    # cs.satcount  # satellite count (#)
    # cs.battery_voltage  # battery voltage (volt)
    # cs.battery_remaining  # battery remaining (%) *
    # cs.armed  # armed state (1:armed, 0:disarmed) *
    # cs.alt_error  # altitude error (m)

    # cs.ax
    # cs.ay
    # cs.az

    # cs.gx
    # cs.gy
    # cs.gz

    # most used values for quick update
    # returns the most up-to-date value
    # does not save the value
    # ---
    def return_latitude(self):
        return cs.lat
    def return_longitude(self):
        return cs.lng
    def return_altitude(self):
        return cs.alt
    def return_vertical_speed(self):
        return cs.verticalspeed
    def return_battery_remaining(self):
        return cs.battery_remaining
    def return_time(self):
        return time.time()
    def return_armed(self):
        return cs.armed
    def return_distance(self):
        return cs.wp_dist
    def return_gps_stat(self):
        return cs.gpsstatus
    def return_mode(self):
        return cs.mode
    def return_ground_speed(self):
        return cs.groundspeed
    def return_gps_count(self):
        return cs.satcount
    def return_roll(self): #
        return cs.roll
    def return_pitch(self): #
        return cs.pitch
    def return_accel_z(self): #
        return cs.az
    def return_gyro_x(self): #
        return cs.gx
    def return_gyro_y(self): #
        return cs.gy
    # ---

    # log sensor data to 'log_data' file
    # note: could add more values
    # Tested: no
    def log_data(self):
        self.log.log_data("- logging sensor data -")
        self.log.log_data("current_time: %f" % self.return_time())
        self.log.log_data("current_mode: " + self.return_mode())
        self.log.log_data("current_altitude: " + str(self.return_altitude()))
        self.log.log_data("current_ground_speed: " + str(self.return_ground_speed()))
        self.log.log_data("current_gps_count: " + str(self.return_gps_count()))
        self.log.log_data("current_armed: " + str(self.return_armed()))
        self.log.log_data("")


#
# Craft class
# ------------------------------------------------------
# Manages craft specific tasks such as
# RC input, way-points, modes, parameters and navigation
#
class Craft:
    sen = Sensors()
    log = Logging()
    con = Config()
    sta = State()

    # RC input [pin, min, max] - check direction
    # default channel maps (1:roll, 2:pitch, 3:throttle, 4:yaw)
    # note: check directions
    rc_throttle = [con.rc_throttle_pin, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
    rc_pitch = [con.rc_pitch_pin, Script.GetParam('RC2_MIN'), Script.GetParam('RC2_MAX')]
    rc_roll = [con.rc_roll_pin, Script.GetParam('RC1_MIN'), Script.GetParam('RC1_MAX')]
    rc_yaw = [con.rc_yaw_pin, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')]

    # ESC output [pin, min, max]
    # motor layout: (1:right, 2:left, 3:front, 4:back)
    esc_f = [con.esc_f_pin, Script.GetParam('RC7_MIN'), Script.GetParam('RC7_MAX')]
    esc_b = [con.esc_b_pin, Script.GetParam('RC8_MIN'), Script.GetParam('RC8_MAX')]
    esc_l = [con.esc_l_pin, Script.GetParam('RC9_MIN'), Script.GetParam('RC9_MAX')]
    esc_r = [con.esc_r_pin, Script.GetParam('RC10_MIN'), Script.GetParam('RC10_MAX')]

    # State variables:
    armed = False

    def __init__(self):
        pass

    # - RC begin -
    # call to engage motor at speed mapped from input of 0 to 1.
    # input:
    #   - channel (ex: rc_throttle)
    #   - double (must be in 0:1 range)
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def rc_set_value(self, chan, in_val):
        # check input
        if in_val > 1 or in_val < 0:
            self.log.log_data("error: rc_set_value() value input out of range.")

            return False
        # map value
        min_val = chan[1]
        max_val = chan[2]
        val = (max_val - min_val)*in_val + min_val
        # set value to channel
        Script.SendRC(chan[0], val, True)
        return True

    # set value to all four RC channels
    # input: double x4 (0:1); to skip a channel input (-1) for its value
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def rc_set_all(self, in_t, in_p, in_r, in_y):
        if in_t != (-1):
            self.rc_set_value(self.rc_throttle, in_t)
        if in_p != (-1):
            self.rc_set_value(self.rc_pitch, in_p)
        if in_r != (-1):
            self.rc_set_value(self.rc_roll, in_r)
        if in_y != (-1):
            self.rc_set_value(self.rc_yaw, in_y)
        return True

    # reset all RC value inputs to idle position
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def rc_reset_all(self):
        # throttle min, pitch half, roll half, yaw half
        self.rc_set_all(0.0, 0.5, 0.5, 0.5)
        return True

    # arm craft.  fail-safes must pass.
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def arm_craft(self):
        self.log.log_data("move class - arming motors")
        self.change_mode_stabilize()
        self.rc_reset_all()
        # left stick to bottom right.  throttle and yaw channels
        self.rc_set_all(0, -1, -1, 1)
        temp_armed = False
        while temp_armed is False:
            temp_armed = self.sen.return_armed()
        self.rc_reset_all()
        self.log.log_data("move class - motors armed")
        self.armed = True
        return True

    # disarm motors (low power mode)
    # Mission critical: no
    # Tested: no
    def disarm_craft(self):
        self.log.log_data("move class - disarming motors")
        self.change_mode_stabilize()
        self.rc_reset_all()
        # left stick bottom left. throttle and yaw channels
        self.rc_set_all(0, -1, -1, 0)
        temp_armed = True
        while temp_armed is True:
            temp_armed = self.sen.return_armed()
        self.rc_reset_all()
        self.rc_reset_all()
        self.log.log_data("move class - motors disarmed")
        self.armed = False
        return True
    # - RC end -

    # - way-points begin -
    # returns the distance between two locations
    # input: start_loc: [lat, lon], end_loc: [lat, lon]
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def calc_distance(self, start_loc, end_loc):
        R=6378137
        # Haversine Formula
        start_loc = [math.radians(start_loc[0]), math.radians(start_loc[1])]
        end_loc = [math.radians(end_loc[0]), math.radians(end_loc[1])]
        dlat = end_loc[0] - start_loc[0]
        dlon = end_loc[1] - start_loc[1]
        a = math.pow(math.sin(dlat/2), 2)+math.cos(start_loc[0])*math.cos(end_loc[0])*math.pow(math.sin(dlon/2), 2)
        c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
        d = R * c
        return d

    # Calculated direction to target from start location
    # Returns direction in degrees
    # Input: start_loc(lat,lon) end_loc(lat,lon)
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def calc_direction_to(self, start_loc, end_loc):
        start_lat = math.radians(start_loc[0])
        start_lon = math.radians(start_loc[1])
        end_lat = math.radians(end_loc[0])
        end_lon = math.radians(end_loc[1])

        dl = end_lon-start_lon
        x = math.sin(dl)*math.cos(end_lat)
        y = math.cos(start_lat)*math.sin(end_lat)-math.sin(start_lat)*math.cos(end_lat)*math.cos(dl)
        b = math.degrees(math.atan2(x, y))
        bn = (b+360) % 360
        return bn

    # returns new location [lat, lon] given current location, displacement and direction
    # input: start_loc: [lat, lon], dist: distance in meters, dir: direction in degrees
    #
    # Mission critical: yes
    # Tested: PASS-ISOLATED-07/21
    def generate_location(self, start_loc, dist, dir):
        # Earths radius, sphere
        R=6378137
        # offsets in meters
        dn = dist*math.cos(math.radians(dir))
        de = dist*math.sin(math.radians(dir))
        # coordinate offsets in radians
        dLat = dn/R
        dLon = de/(R*math.cos(math.pi*start_loc[0]/180))
        # offsetPosition, decimal degrees
        new_lat = start_loc[0] + dLat * 180/math.pi
        new_lon = start_loc[1] + dLon * 180/math.pi
        return new_lat, new_lon

    # passing all ([lat,lng],alt) values will send this way-point to craft
    # input: [latitude, longitude], (int) altitude (units???)
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def set_waypoint(self, loc, loc_alt):
        new_wp = MissionPlanner.Utilities.Locationwp()  # create way-point object
        MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp, loc[0])  # set way-point latitude
        MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp, loc[1])  # set way-point longitude
        MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp, loc_alt)  # set way-point altitude
        self.change_mode_guided()
        MAV.setGuidedModeWP(new_wp)  # begin way-point
        # logs way-point info
        self.log.log_data("way-point set: lat:" + str(loc[0]) +
            " lng:" + str(loc[1]) + " alt:" + str(loc_alt))

    # Returns after current way-point complete based on distance to target.
    # note: blocking functions.  could add timeout
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def wait_waypoint_complete(self):
        self.log.log_data("move class - waiting for way-point")
        time.sleep(1)
        flying_to_wp = True
        while flying_to_wp:
            if self.sen.return_distance() < self.con.waypoint_tolerance:
                flying_to_wp = False
        self.log.log_data("move class - wait: way-point complete: " + str(self.sen.return_distance()))
        return True

    # Navigation manager determines path and sets way-points
    # Returns after target is reached.  Holds position at target
    # input: target: [lat, lon], target_alt: target altitude
    # note: add types like directional or decent
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def navigation_manager(self, target, target_alt):
        self.log.log_data("move class - navigation start")
        total_dist = self.calc_distance([self.sen.return_latitude(), self.sen.return_longitude()], target)
        total_alt = math.fabs(self.sen.return_altitude() - target_alt)
        nav_complete = False
        while nav_complete == False:
            current_loc = [self.sen.return_latitude(), self.sen.return_longitude()]
            current_alt = self.sen.return_altitude()
            remaining_dist = self.calc_distance(current_loc, target)
            self.log.log_data("move class - navigation: distance: " + str(remaining_dist))
            # find next alt
            if current_alt > (target_alt + self.con.jump_alt):
                # jump altitude
                temp_alt = (current_alt - self.con.jump_alt)
            else:
                # at target altitude
                temp_alt = target_alt
            # find next loc
            if abs(remaining_dist) > self.con.jump_distance:
                # find next jump location
                temp_dir = self.calc_direction_to(current_loc, target)
                temp_jump_loc = self.generate_location(current_loc, self.con.jump_distance, temp_dir)
                self.log.log_data("move class - navigation: next way-point")
                self.set_waypoint(temp_jump_loc, temp_alt)
                self.wait_waypoint_complete()
            else:
                # next jump is target
                self.log.log_data("move class - moving to final way-point")
                self.set_waypoint(target, target_alt)
                self.wait_waypoint_complete()
                nav_complete = True
        self.log.log_data("move class - navigation complete")
    # - way-points end -

    # - modes begin -
    # waits for landing based on vertical speed
    # note: blocking function. could add timeout
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def wait_for_land(self):
        self.log.log_data("move class - waiting for land")
        # wait for craft to begin landing
        time.sleep(2)
        landing = True
        while landing:
            # craft vertical speed within touchdown range
            if self.sen.return_vertical_speed() < 0.0 and self.sen.return_vertical_speed() > self.con.desired_vert_speed:
                landing = False
        temp_armed = True
        # wait for auto disarm
        while temp_armed:
            temp_armed = self.sen.return_armed()
        self.change_mode_guided()
        State.landed = True
        self.log.log_data("move class - landing complete")
        return True

    # change to landing mode
    # does not return until landing complete (or timeout - NI)
    # Mission critical: yes
    # Tested: PASS-SITL-07/26
    def change_mode_landing(self):
        Script.ChangeMode('LAND')  # set mode to LAND
        self.wait_for_land()

    # change mode to loiter
    # info: hold position and altitude
    # Mission critical: yes
    # Tested: no
    def change_mode_loiter(self):
        Script.ChangeMode('LOITER')
        # Script.WaitFor('LOITER', 5000)  # whats this do?

    # change mode to stabilize
    # info: self-levels the roll and pitch axis.  allows RC input
    # Mission critical: ~
    # Tested: no
    def change_mode_stabilize(self):
        Script.ChangeMode('STABILIZE')

    # change mode to guided
    # info: auto way-point mode
    # Mission critical: yes
    # Tested: PASS-SITL-07/26
    def change_mode_guided(self):
        Script.ChangeMode('GUIDED')
        # Script.WaitFor('GUIDED', 5000)

    # return to launch site and land
    # Mission critical: no
    # Tested: no
    def change_mode_rtl(self):
        Script.ChangeMode('RTL')
        # Script.WaitFor('RTL', 5000)

    # takeoff for testing and non-assisted flying
    # do not return until takeoff completes (or timeout - NI)
    # note: desired vertical speed is 2
    # Mission critical: no
    # Tested: PASS-SITL-07/26
    def change_mode_takeoff(self):
        self.log.log_data("move class - begin takeoff")
        temp_start_alt = self.sen.return_altitude()
        # arm
        self.arm_craft()
        # enter loiter mode
        self.change_mode_stabilize()
        # engage motors then throttle at 80%
        self.rc_reset_all()
        self.rc_set_value(self.rc_throttle, self.con.takeoff_throttle_val)
        time.sleep(1)
        taking_off = True
        self.log.log_data("move class - going up")
        self.log.log_data("move class - waiting for altitude")
        # maintain until at set altitude
        while taking_off:
            # monitor vertical speed
            if self.sen.return_vertical_speed() < self.con.default_takeoff_speed:
                self.con.takeoff_throttle_val + 0.01
            if self.sen.return_vertical_speed() > self.con.default_takeoff_speed+1:
                self.con.takeoff_throttle_val - 0.01
            # set throttle
            self.rc_set_value(self.rc_throttle, self.con.takeoff_throttle_val)
            # update altitude
            if self.sen.return_altitude() > (temp_start_alt + self.con.default_takeoff_alt):
                taking_off = False
        self.change_mode_guided()
        self.set_waypoint([self.sen.return_latitude(), self.sen.return_longitude()], self.con.default_takeoff_alt)
        self.rc_reset_all()
        State.landed = False
        self.log.log_data("move class - takeoff complete")
    # - modes end -

    # - parameters begin -
    # RC channels mapped to ESCs to control individual motor speeds
    # ToDo: incomplete FIX
    # Mission critical: ~
    # Tested: no
    def params_rc_setup():
        Script.ChangeParam("RC7_FUNCTION", 33)
        Script.ChangeParam("RC8_FUNCTION", 34)
        Script.ChangeParam("RC9_FUNCTION", 35)
        Script.ChangeParam("RC10_FUNCTION", 36)

    # Sets default flight configuration PID values.
    # ToDo: incomplete FIX
    # Mission critical: yes
    # Tested: no
    def params_pids_default(self):
        pass  # Script.ChangeParam()

    # Enables and disables the GPS depending on boolean input.
    # Input: in_en(True:enable, False:disable)
    # Mission critical: yes
    # Tested: no
    def params_gps(self, in_en):
        if in_en:
            # enable GPS
            self.change_mode_stabilize()
            Script.ChangeParam("AHRS_GPS_USE", 1)
            State.gps_enabled = True
            self.log.log_data("Craft class - GSP enabled")
        else:
            # disable GPS
            Script.ChangeParam("AHRS_GPS_USE", 0)
            State.gps_enabled = False
            self.log.log_data("Craft class - GSP disabled")

    # Sets the crafts fail-safes at script start.
    # Fail-safes: battery, ground-station, RC connection, Kalman filter (EKF), EKF threshold
    # Mission critical: yes
    # Tested: no
    def params_failsafe_setup(self):
    # BRD_SAFETYENABLE 0 # button off
        Script.ChangeParam("FS_BATT_ENABLE", 0)  # 1:land if low battery, 0:disable (don't stop till you drop) maybe
        Script.ChangeParam("FS_GCS_ENABLE", 0)  # 0:disabled (local ground station?)
        Script.ChangeParam("FS_THR_ENABLE", 0)  # 0:disabled (no radio for mission)
        Script.ChangeParam("FS_EKF_ACTION", 1)  # 1:land
        Script.ChangeParam("FS_EKF_THRESH", 1.0)  # 1:relaxed (probably needed with rough flight)

    # Setup parameters for flight at script start.
    # Disable the fence, call 'params_failsafe_setup()', *disable GPS
    # ToDo: ensure all parameters are covered.  Possibly set redundant parameters as well.
    # Mission critical: yes
    # Tested: no
    def params_setup(self):
        Script.ChangeParam("FENCE_ENABLE", 0)  # 0:disabled
        self.params_failsafe_setup()
        if self.con.disable_gps_on_start:
            self.params_gps(False)
    # - parameters end -


    # Monitor on-board battery and output periodic updates to log
    # ToDo: add to mission loop
    # Mission critical: no
    # Tested: no
    def battery_monitor(self):
        temp_bat_remaining = self.sen.return_battery_remaining()
        if (self.sta.last_battery == -1):
            State.last_battery = temp_bat_remaining
        if (temp_bat_remaining < self.sta.last_battery - battery_update_interval):
            self.log.log_data("move class - battery remaining: " + str(temp_bat_remaining))


    # Runs at script start.  Checks basic current craft setup.
    # Will pass if the craft is setup correctly for script control.
    # Checks: *armed, location, GPS lock
    # ToDo: check will fail is no GPS lock.  MA_01 no GPS on start.  FIX
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def check_ready(self):
        check_pass = True
        # armed check
        if self.sen.return_armed() and self.con.require_disarm:
            check_pass = False
            self.log.log_data("check_ready - error: craft armed")
        # location check
        if self.con.location == "dam" or self.con.location == "brd": pass # load values into location variable?
        else:
            check_pass = False
            self.log.log_data("check_ready - error: location not recognized")
        # GPS lock
        if self.sen.return_gps_stat() != 3:
            check_pass = False
            self.log.log_data("check_ready - error: GPS fail")
        # return
        return check_pass

    # Setup general at script start.
    # Calls: check_ready(), rc_reset_all(), change_mode_guided()
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def setup(self):
        if self.check_ready() is False:
            self.log.log_data("move class - error: check_ready() failed")
            return False
        self.rc_reset_all()
        self.params_setup()
        self.change_mode_guided()
        self.log.log_data("move class - setup complete")
        return True


#
# Rocket Class
# ------------------------------------------------------
# Rocket Class manages rocket specific tasks.
# Detect rocket launch, payload release, and recovery.
#
class Rocket:
    sen = Sensors()
    cra = Craft()
    log = Logging()
    con = Config()
    sta = State()

    def __init__(self):
        pass

    # If craft is falling but not launched or released.
    # Talk to frame team about adding a switch to the arm that contacts the payload can.
    # - This will allow the script to act quickly after and have no doubt that the payload has been released.

    # Monitors altitude and returns once current altitude has increased by 'launch_trigger_altitude'
    # Note: if signal loss, it will return once reconnected (test)
    # Mission critical: yes
    # Tested: no
    def wait_for_launch(self):
        self.log.log_data("rocket class - wait for launch")
        starting_alt = self.sta.start_alt[0]
        launched = False
        while launched == False:
            if self.sen.return_altitude() > (starting_alt + self.con.launch_trigger_altitude):
                launched = True
        State.start_time[1] = self.sen.return_time()
        # GPS disabled
        State.start_alt[1] = self.sen.return_altitude()
        State.rocket_launced = True
        self.log.log_data("rocket class - launch detected")

    # Returns once payload release is detected
    # Waits for falling altitude change over time (will return after possible loss of telemetry)
    # GPS will be disabled during this state and enabled once complete
    # Mission critical: yes
    # Tested: no
    def wait_for_payload_release(self):
        self.log.log_data("rocket class - payload release wait begin")
        if self.con.pr_use_alt:
            # altitude trigger
            self.log.log_data("rocket class - pr using altitude trigger")
            testing_release = True
            while testing_release:
                temp_cur_alt = self.sen.return_altitude()
                time.sleep(pr_alt_sleep_test)
                # decend 'pr_alt_trigger' in 'pr_alt_sleep_test' seconds
                if self.sen.return_altitude() < (temp_cur_alt-self.con.pr_alt_trigger):
                    testing_release = False
            State.rocket_payload_released = True
        else:
            # switch trigger
            pass
        if self.sen.disable_gps_on_start: self.cra.params_gps(True);  # enable GPS
        self.log.log_data("rocket class - payload release wait complete")

    # Recovery craft after rocket release
    # Mission critical: yes
    # Tested: no
    def recover(self):
        # assumed terminal velocity: 35 to 40 m/s
        # outline:
        # - assume level flight from CG
        # - set a way-point just below current position and to the side to avoid prop wash
        self.log.log_data("rocket class - recovery start")
        recovery_attemp = True
        while recovery_attemp:
            # get new target for recovery FIX
            self.cra.set_waypoint([self.sen.return_latitude(), self.sen.return_longitude()], (self.sen.return_altitude()-self.con.recover_wp_below))
            # wait for recovery
            if (self.con.wait_recov):
                self.log.log_data("rocket class - wait for recovery")
                current_vs = self.sen.return_vertical_speed()
                starting_time = self.sen.return_time()
                while abs(current_vs) > abs(wr_vs_max):
                    current_vs = self.sen.return_vertical_speed()
                    if starting_time+self.sen.return_time() > self.con.wr_retry_time:
                        # timeout
                        self.log.log_data("rocket class - recovery timeout")
                        time.sleep(wr_retry_time)
                        starting_time = self.sen.return_time()
                        self.log.log_data("rocket class - recovery retry")
                self.log.log_data("rocket class - craft recovered")
            else:
                time.sleep(4)
        self.log.log_data("rocket class - recovery complete")


#
# Testing class
# ------------------------------------------------------
# Testing functions for different aspects of mission
class Testing:
    sen = Sensors()
    log = Logging()
    cra = Craft()
    con = Config()
    rok = Rocket()

    def __init__(self):
        pass

    # test file operations
    #
    # Mission critical: no
    # Tested: no
    def test_filelog(self):
        self.log.log_data("testing class - test_filelog() begin")
        log.log_data_custom("test_log", "test log data. \
            test_log_empty should be empty")
        log.log_data_custom("test_log_empty", "test log.  if here then FAIL")
        log.clear_log("test_log_empty")
        self.log_data("test_filelog - note: check for 'test_log' and '_empty'")
        self.log.log_data("testing class - test_filelog() complete")

    # test sensor class
    #
    # Mission critical: no
    # Tested: no
    def test_sensors(self):
        self.log.log_data("testing class - test_sensors() begin")
        self.log.log_data("")
        for num in range(3):
            self.sen.log_data()
            time.sleep(1)
        self.log.log_data("testing class - test_senssors() complete")

    # test craft class distance and direction functions
    # FIX
    # Mission critical: no
    # Tested: no
    def test_navigation_sub_functions(self):
        pass

    # test arm and disarm - optional disarm (not tested)
    #
    # Mission critical: no
    # Tested: no
    def test_arm(self):
        # arm
        self.log.log_data("testing class - test_arm() begin")
        self.log.log_data("testing class - arming craft")
        self.cra.arm_craft()
        if self.con.test_disarm is True:
            # print info user action
            self.log.log_data("testing class - note: flex throttle a bit to prevent auto disarm")
            self.log.log_data("testing class - disarming in 20 seconds")
            time.sleep(16)
            self.log.log_data("testing class - disarming in 4 seconds")
            time.sleep(4)
            # disarm
            self.log.log_data("testing class - disarming craft")
            self.cra.disarm_craft()
            self.log.log_data("testing class - test_arm() complete")

    # test takeoff
    #
    # Mission critical: no
    # Tested: PASS-SITL-07/26
    def test_takeoff(self):
        self.log.log_data("test_takeoff - begin")
        if self.con.include_takeoff_t:
            # takeoff
            self.log.log_data("test_takeoff - taking off")
            self.cra.change_mode_takeoff()
            self.log.log_data("test_takeoff - wait " + str(self.con.hold_position_time) + " seconds")
            time.sleep(self.con.hold_position_time)
            self.log.log_data("move class - takeoff complete")
        else:
            self.log.log_data("test_takeoff - skip takeoff")
        # land
        self.log.log_data("test_takeoff - landing")
        self.cra.change_mode_landing()
        self.cra.wait_for_land()
        self.log.log_data("test_takeoff - complete")

    # test way-points
    #
    # Mission critical: no
    # Tested: no
    def test_waypoints(self):
        self.log.log_data("test_waypoints - begin")
        if self.con.include_takeoff_wp:
            self.log.log_data("test_waypoints - taking off")
            self.cra.change_mode_takeoff()
        else:
            self.log.log_data("test_waypoints - note: ensure craft is flying already")
            self.log.log_data("test_waypoints - beginning in 6 seconds")
            time.sleep(6)
        # move to way-point 1
        self.log.log_data("test_waypoints - move to way-point one")
        if self.con.location == "dam":
            self.log.log_data("test_waypoints - Damonte " + str(self.con.wp_1_index))
            self.cra.set_waypoint(self.con.loc_dam[self.con.wp_1_index], self.con.testing_altitude)
            self.cra.wait_waypoint_complete()
        # way-pint 2
        self.log.log_data("test_waypoints - move to way-point two")
        if self.con.location == "dam":
            self.log.log_data("test_waypoints - Damonte " + str(self.con.wp_2_index))
            self.cra.set_waypoint(self.con.loc_dam[self.con.wp_2_index], self.con.testing_altitude)
            self.cra.wait_waypoint_complete()
        # RTL
        if self.con.return_after:
            self.log.log_data("test_waypoints - RTL and land")
            self.cra.change_mode_rtl()
            self.cra.wait_waypoint_complete()
        self.log.log_data("test_waypoints - complete")

    # test recovery
    #
    # Mission critical: no
    # Tested: no
    def test_recovery(self):
        self.log.log_data("test_recovery - begin")
        if (self.con.location != "dam"):  # location check
            self.log.log_data("test_recovery - location error")
            return False
        if (self.con.takeoff_before_recover):  # takeoff
            self.cra.change_mode_takeoff()
        if (self.con.flyto_recover):
            self.log.log_data("test_recovery - flying to start position")
            self.cra.set_waypoint(self.con.loc_dam[5], self.con.test_recover_start_alt)  # recovery location
            self.cra.wait_waypoint_complete()
        self.log.log_data("test_recovery - disabling craft")
        self.cra.change_mode_stabilize()
        self.cra.rc_set_value(self.cra.rc_throttle, 0.5)
        time.sleep(1)
        self.cra.rc_reset_all()  # cut throttle
        time.sleep(self.con.recover_test_sleep)  # wait
        self.log.log_data("test_recovery - starting recovery")
        self.rok.recover()  # recover
        if (self.con.fly_back_home):
            self.log.log_data("test_recovery - flying back")
            self.cra.navigation_manager(self.con.loc_dam[0], 20)
            self.cra.change_mode_landing()

    # test navigation
    #
    # Mission critical: no
    # Tested: PASS-SITL-08/18
    def test_navigation(self):
        self.log.log_data("test_navigation - begin")
        self.cra.navigation_manager(con.loc_rand_unr, 20)
        self.log.log_data("test_navigation - end")

    # test movement speeds (ground and vertical speeds)
    # constant altitude, constant position
    # test optimal speeds with same altitude and position change
    # Mission critiacal: no
    # Tested: PASS-SITL-08/18
    def test_movement_speeds(self):
        self.log.log_data("test_movement_speeds - begin")
        temp_max_ground_speed = 0
        temp_max_vertical_speed = 0
        temp_optimal_ground_speed = 0
        temp_optimal_vertical_speed = 0
        # settings begin
        temp_test_alt = 20  # operation altitude most cases
        temp_test_alt_offset = 30  # test 2 - distance above temp_test_alt
        temp_start_wp = 2  # all tests
        temp_end_wp = 5  # test 1 and 3
        temp_offset_dist = 6  # test 2 target way-point offset
        temp_offset_dir = 270  # test 2
        # settings end
        # test constant altitude (1) - max ground speed
        self.cra.set_waypoint(self.con.loc_dam[temp_start_wp], temp_test_alt)  # set starting way-point
        self.cra.wait_waypoint_complete()
        self.cra.set_waypoint(self.con.loc_dam[temp_end_wp], temp_test_alt)  # set target way-point
        temp_start_dist = self.cra.calc_distance([self.sen.return_latitude(), self.sen.return_longitude()], self.con.loc_dam[temp_end_wp])
        temp_current_dist = temp_start_dist
        while temp_current_dist > (temp_start_dist/2):  # wait for middle of way-point
            temp_current_dist = self.cra.calc_distance([self.sen.return_latitude(), self.sen.return_longitude()], self.con.loc_dam[temp_end_wp])
        temp_max_ground_speed = self.sen.return_ground_speed()  # get ground speed
        # test constant position (mostly)(2) - max vertical speed
        self.cra.set_waypoint(self.con.loc_dam[temp_start_wp], (temp_test_alt+temp_test_alt_offset))
        self.cra.wait_waypoint_complete()
        self.cra.set_waypoint(self.cra.generate_location(self.con.loc_dam[temp_start_wp], temp_offset_dist, temp_offset_dir), temp_test_alt)
        temp_start_alt = self.sen.return_altitude()
        temp_current_alt = temp_start_alt
        while temp_current_alt > (temp_start_alt/2):
            temp_current_alt = self.sen.return_altitude()
        temp_max_vertical_speed = self.sen.return_vertical_speed()
        # # test optimal ground and vertical speed (3) FIX
        # self.cra.set_waypoint(self.con.loc_dam[temp_start_wp], (temp_start_alt+temp_start_dist))
        # self.log.log_data("alt offset/wp dist: " + str(temp_start_dist))
        # self.cra.wait_waypoint_complete()
        # self.cra.set_waypoint(self.con.loc_dam[temp_end_wp], temp_start_alt)
        # temp_current_dist = temp_start_dist
        # while (temp_current_alt-temp_start_alt) > (temp_start_dist/2):
            # temp_current_dist = self.cra.calc_distance([self.sen.return_latitude(), self.sen.return_longitude()], self.con.loc_dam[temp_end_wp])
        # temp_optimal_ground_speed = self.sen.return_ground_speed()
        # temp_optimal_vertical_speed = self.sen.return_vertical_speed()
        # test complete - log info
        self.log.log_data("RESULTS")
        self.log.log_data("test 1 - max groundspeed: " + str(temp_max_ground_speed))
        self.log.log_data("test 2 - max verticalspeed: " + str(temp_max_vertical_speed))
        # self.log.log_data("test 3 - optimal groundspeed: " + str(temp_optimal_ground_speed))
        # self.log.log_data("test 3 - optimal groundvertical: " + str(temp_optimal_vertical_speed))
        self.log.log_data("RESULTS end")
        self.log.log_data("--Please email William the file located in Documents\CODE_LOGS--")
        self.log.log_data("test_movement_speeds - end")


#
# Mission Class
# ------------------------------------------------------
# setup for tests of minor classes (Logging, Sensors, Move)
# note:
#     [launch, eject, recover, navigate, land]
#
class Mission:
    cra = Craft()
    sen = Sensors()
    log = Logging()
    tes = Testing()
    con = Config()
    roc = Rocket()
    sta = State()

    def __init__(self):
        pass

    # reset all variables
    #
    # Mission critical: yes
    # Tested: no
    def reset_values(self):
        # state variables
        # start, launch, eject, recover, land
        State.start_time = [0, 0, 0, 0, 0]
        State.start_pos = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        State.start_alt = [0, 0, 0, 0, 0]
        State.landed = False
        State.mission_begin = False
        State.mission_complete = False
        self.log.log_data("mission class - variables reset")

    # setup at start
    #
    # Mission critical: yes
    # Tested: no
    def setup(self):
        self.reset_values()
        # set state variables
        State.mission_begin = True
        State.start_time[0] = self.sen.return_time()
        State.start_pos[0] = [self.sen.return_latitude(), self.sen.return_longitude()]
        State.start_alt[0] = self.sen.return_altitude()
        # disable gps (optional)
        if self.con.disable_gps_on_start: self.cra.params_gps(False)
        self.log.log_data("mission class - setup complete")


    # wrap-up at end of mission alpha
    #
    # Mission critical: yes
    # Tested: no
    def end_mission(self):
        State.mission_complete = True

    # Mission Alpha 01 - ARLISS main mission run function
    #
    # Mission critical: yes
    # Tested: no
    def ma_01(self):
        self.log.log_data("MA_01 begin")
        # wait for rocket launch while idle
        self.roc.wait_for_launch()
        # wait for payload release
        self.roc.wait_for_payload_release()
        # recovery phase
        self.roc.recover()
        # navigation phase
        self.cra.navigation_manager(self.con.loc_brd_target, State.start_alt[0])
        # landing phase
        self.cra.change_mode_landing()
        # mission complete
        self.end_mission()
        self.log.log_data("MA_01 complete")


    # run mission
    #
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def run_mission(self):
        self.log.log_data("mission class - running mission")
        if (self.con.preset_config != "NA"):
            if (self.con.preset_config == "ma-01"):
                self.log.log_data("mission class - running preset mission ma_01")
                self.con.config_default_ma01()
                self.ma_01()
            elif (self.con.preset_config == "dam_t4"):
                self.log.log_data("mission class - running preset mission dam_t4")
                self.con.config_default_t4()
                self.tes.test_recovery()
            elif (self.con.preset_config == "dam_t5"):
                self.log.log_data("mission class - running preset mission dam_t5")
                self.con.config_default_t5()
                self.tes.test_navigation()
            elif (self.con.preset_config == "dam_t6"):
                self.log.log_data("mission class - running preset mission dam_t6")
                self.con.config_default_t6()
                self.tes.test_movement_speeds()
            else:
                self.log.log_data("mission - run_mission error: preset_config unknown")
                return False
        elif (self.con.mission_mode == "0"):  # no mission
            self.log.log_data("mission class - no mission to run")
            self.log.log_data("mission class - exiting")
            return True
        elif (self.con.mission_mode == "t1"):  # test arm / disarm
            self.log.log_data("mission class - running mission t1")
            self.tes.test_arm()
        elif (self.con.mission_mode == "t2"):  # test takeoff / landing
            self.log.log_data("mission class - running mission t2")
            self.tes.test_takeoff()
        elif (self.con.mission_mode == "t3"):  # test way-points
            self.log.log_data("mission class - running mission t3")
            self.tes.test_waypoints()
        elif (self.con.mission_mode == "t4"):  # test recovery
            self.log.log_data("mission class - running mission t4")
            self.tes.test_recovery()
        elif (self.con.mission_mode == "t5"):  # test navigation
            self.log.log_data("mission class - running mission t5")
            self.tes.test_navigation()
        elif (self.con.mission_mode == "t6"):  # test movement speeds
            self.log.log_data("mission class - running mission t6")
            self.tes.test_movement_speeds()
        elif (self.con.mission_mode == "ma-01"):  # mission ma-01
            self.log.log_data("mission class - running mission ma-01")
            self.ma_01()
        else:
            self.log.log_data("mission class - run_mission error: mission_mode unknown")
            return False  # exit
        self.log.log_data("mission class - mission complete")
        return True

    # Call to start the script
    # Mission critical: yes
    # Tested: PASS-SITL-08/18
    def autorun(self):
        if self.cra.setup() is True:
            self.log.log_data("mission class - craft check passed.  starting script")
            self.setup()
            self.run_mission()
            return True
        else:
            self.log.log_data("mission class - error: craft did not pass checks.  fix and try again.")
            return False


# autostart code
log = Logging()
con = Config()

if (con.run_test):
    log.log_data("run_test begin")
    tes = Testing()
    tes.test_filelog()
    tes.test_sensors()
    log.log_data("run_test complete")
else:
    # start script
    log.log_data("script begin")
    log.log_data("-------------")
    mission = Mission()
    mission.autorun()
    log.log_data("---------------")
    log.log_data("script complete")
