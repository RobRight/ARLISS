import math  # math
import os  # log file path
import time  # sleep and current time
import clr  # AddReference
clr.AddReference("MissionPlanner")
clr.AddReference("MissionPlanner.Utilities")
clr.AddReference("MAVLink")  # includes the Utilities class
import MissionPlanner
from MissionPlanner.Utilities import Locationwp
import MAVLink

# mission options:
# -----------------
# (0) no mission
# (t1) test arm and disarm - arms on ground, seconds later disarms.
# (t2) test takeoff and landing - goes up and lands at the same location.
# (t3) test waypoints - flys to a few waypoints, then rtl and land
# (t4) (NI) test recovery (NI) - not implemented
# (ma-01) - mission_alpah - complete mission from idle, launch, recovery, navigation, and landling.

# location options:
# -----------------
# (dem) Demonte Ranch
# (brd) (NI) Black Rock Desert (NI) - not implemented

class Config:
    # - general settings -
    verbose = True
    mission_mode = "t2"
    location = "dem"
    run_test = False # sensor and file testing
    require_disarm = False
    ## custom_path = [0,1,2,3,4,5,6,7,8,9,10]
    # ---------------------------
    # - testing class -
    # test_arm function
    test_disarm = False  # not implemented
    # test_waypoints function
    include_takeoff = False  # also test_takeoff --
    testing_altitude = 30  # units
    wp_1_index = 2  # test_waypoint loc 1
    wp_2_index = 5
    return_after = True
    # - logging class -
    log_enable = True  # enable file logging
    print_enable = True  # enable console logging
    default_name = "log_file"  # file log filename prefix
    # - move class -
    # min distance from waypoint before moving on
    waypoint_tolerance = 2  # m ??
    default_takeoff_alt = 20  # m ??
    default_takeoff_speed = 2  # m/s ??
    # rc pins
    rc_throttle_pin = 3
    rc_pitch_pin = 2
    rc_roll_pin = 1
    rc_yaw_pin = 4
    # esc pins
    esc_fr_pin = 0
    esc_br_pin = 0
    esc_bl_pin = 0
    esc_fl_pin = 0
    # ---------------------------
    # locations: [latitude, longitude]
    # demonte rance
    loc_dem = [
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


#
# Logging Class
# ------------------------------------------------------
#
class Logging:
    con = Config()
    directory = ''  # path to user directory
    start_time = ()  # assigned at start by 'Mission' class

    # set directory and create folder if not found
    def __init__(self):
        # TEMP
        self.start_time = time.localtime()
        # http://askubuntu.com/questions/138922/path-to-user-desktop-using-python
        self.directory = os.path.expanduser('~') + "\Documents\CODE_LOGS"
        # http://stackoverflow.com/questions/273192/how-to-check-if-a-directory-exists-and-create-it-if-necessary
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    # generate a file name directory and time stanp
    def generate_filename(self, given):
        filename = given + "_" + time.strftime("%m_%d_%y__%H_%M_%S", self.start_time)
        filename2 = os.path.join(self.directory, filename + ".txt")
        return filename2

    # clear given file
    def clear_log(self, in_f_name):
        if self.con.log_enable:
            filename = self.generate_filename(in_f_name)
            f = open(filename, 'w')
            f.close()

    # write given data to file
    def log_data(self, in_data):
        if self.con.log_enable:
            filename = self.generate_filename(self.con.default_name)
            f = open(filename, 'a')
            f.write(in_data + "\n")
            f.close()
        if self.con.print_enable is True:
            print(in_data)
    
    # write given data to file with given name
    def log_data_custom(self, in_name, in_data):
        if self.con.log_enable:
            filename = self.generate_filename(in_name)
            f = open(filename, 'a')
            f.write(in_data + "\n")
            f.close()
        if self.com.print_enable is True:
            print(in_name + ": " + in_data)


# Sensors Class
# ------------------------------------------------------
# call get_data() to update sensor readings
#
class Sensors:
    log = Logging()

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
        ## self.log.log_data("logging class - got new sensor data")

        self.current_time = time.time()
        self.current_mode = cs.mode  # NOTE
        self.current_distance = cs.wp_dist  # NOTE
        self.current_waypoint = cs.wpno
        self.current_altitude = cs.alt  # NOTE
        self.current_vertical_speed = cs.verticalspeed  # NOTE
        self.current_ground_speed = cs.groundspeed  # NOTE
        self.current_ground_course = cs.groundcourse
        self.current_wind_direction = cs.wind_dir
        self.current_wind_speed = cs.wind_vel
        self.current_roll = cs.roll  # NOTE
        self.current_pitch = cs.pitch  # NOTE
        self.current_yaw = cs.yaw
        self.current_lat = cs.lat
        self.current_lng = cs.lng
        self.current_gps_stat = cs.gpsstatus
        self.current_gps_count = cs.satcount
        self.current_battery_voltage = cs.battery_voltage
        self.current_armed = cs.armed  # NOTE
        self.current_altitude_error = cs.alt_error

        self.current_accel = []
        self.current_accel.append(cs.ax)
        self.current_accel.append(cs.ay)
        self.current_accel.append(cs.az)

        self.current_gyro = []
        self.current_gyro.append(cs.gx)
        self.current_gyro.append(cs.gy)
        self.current_gyro.append(cs.gz)
    
    # log sensor data to 'log_data' file
    def log_data(self):
        self.log.log_data("- logging sensor data -")
        self.log.log_data("current_time: %f" % self.current_time)
        self.log.log_data("current_mode: " + self.current_mode)
        self.log.log_data("current_altitude: " + str(self.current_altitude))
        self.log.log_data("current_ground_speed: " + str(self.current_ground_speed))
        self.log.log_data("current_gps_count: " + str(self.current_gps_count))
        self.log.log_data("current_armed: " + str(self.current_armed))
        self.log.log_data("")


# Move class
# ------------------------------------------------------
# waypoints, land, arm/disarm
# note: need loiter function; check channel mappings
#
class Move:
    sen = Sensors()
    log = Logging()
    con = Config()

    # RC input [pin, min, max] - check direction
    rc_throttle = [con.rc_throttle_pin, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
    rc_pitch = [con.rc_pitch_pin, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')]  # B
    rc_roll = [con.rc_roll_pin, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX')]
    rc_yaw = [con.rc_yaw_pin, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX')]
    # ESC output [pin, min, max]
    # esc_fr = [esc_fr_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_br = [esc_br_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_bl = [esc_bl_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_fl = [esc_fl_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]

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

    # set value to all four rc channels
    # input: double x4 (0:1); to skip a channel input (-1) for its value
    # pass: NA
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

    # reset all rc value inputs to idle position
    # pass: NA
    def rc_reset_all(self):
        # throttle min, pitch half, roll half, yaw half
        self.rc_set_all(0.0, 0.5, 0.5, 0.5)
        return True

    # arm craft.  failsafes must pass.
    # pass: NA
    def arm_craft(self):
        self.log.log_data("move class - arming motors")
        self.change_mode_loiter()
        self.rc_reset_all()
        # left stick to bottom right.  throttle and yaw channels
        self.rc_set_all(0, -1, -1, 1)
        Script.WaitFor('ARMING MOTORS', 5000)
        self.rc_reset_all()
        ## if (self.verbose):
        self.log.log_data("move class - motors armed")
        self.armed = True
        return True

    # disarm motors (low power mode)
    # pass: NA
    def disarm_craft(self):
        self.log.log_data("move class - disarming motors")
        self.change_mode_loiter()
        self.rc_reset_all()
        # left stick bottom left. throttle and yaw channels
        self.rc_set_all(0, -1, -1, 0)
        Script.WaitFor('DISARMING MOTORS', 5000)
        self.rc_reset_all()
        ## if (self.verbose):
        self.log.log_data("move class - motors disarmed")
        self.armed = False
        return True
    # - rc end -

    # - waypoints begin -
    # passing all ([loc,lng],alt) values will send this waypoint to craft
    # input: [latitude, longitude], (int) altitude (units???)
    # pass: NA
    def set_waypoint(self, loc, loc_alt):
        # http://www.diydrones.com/forum/topics/mission-planner-python-script
        new_wp = MissionPlanner.Utilities.Locationwp()  # create waypoint object
        MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp, loc[0])  # set waypoint latitude
        MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp, loc[1])  # set waypoint longitude
        MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp, loc_alt)  # set waypoint altitude
        MAV.setGuidedModeWP(new_wp)  # begin waypoint
        # prints waypoint info.
        self.log.log_data("waypoint set: lat:" + str(loc[0]) +
            " lng:" + str(loc[1]) + " alt:" + str(loc_alt))

    # waits for waypoint_complete() to finish
    # note: blocking functions.  could add timeout
    # pass: NA
    def wait_waypoint_complete(self):
        self.log.log_data("move class - waiting for waypoint")
        time.sleep(1)
        flying_to_wp = True
        while flying_to_wp:
            self.sen.get_data()
            if self.sen.current_distance < self.con.waypoint_tolerance:
                flying_to_wp = False
        self.log.log_data("move class - wait: waypoint complete: " + str(self.sen.current_distance))
    # - waypoints end -

    # - modes begin -
    # engage landing
    # do not return until landing complete or timeout
    # pass: NA
    def change_mode_landing(self):
        # http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_land
        # MAV_CMD_NAV_LAND(0,0,0,0,l,l) - NODE: do not know if this command works
        Script.ChangeMode('LAND')  # set mode to LAND
        # http://copter.ardupilot.com/wiki/land-mode/
    
    def wait_for_land(self):
        landing = True
        while landing:
            self.sen.get_data()
            if self.sen.current_vertical_speed < 0.0 and self.sen.current_vertical_speed > -0.1:
                landing = False
        self.log.log_data("move class - wait: landing complete")
        return True

    # change mode to loiter
    # info: hold position and altitude
    # pass: NA
    def change_mode_loiter(self):
        Script.ChangeMode('LOITER')
        # Script.WaitFor('LOITER', 5000)  # whats this do?
    
    # change mode to guided
    # info: 
    # pass: NA
    def change_mode_guided(self):
        Script.ChangeMode('GUIDED')
        # Script.WaitFor('GUIDED', 5000)

    # return to launch site and land
    # pass: NA
    def change_mode_rtl(self):
        Script.ChangeMode('RTL')
        # Script.WaitFor('RTL', 5000)

    # takeoff for testing and non-assisted flying
    # do not return until takeoff completes or timeout
    # pass: NA - not ready
    def change_mode_takeoff(self):
        pass
        '''
        takeoff_alt = self.con.default_takeoff_alt
        takeoff_speed = self.con.default_takeoff_speed
        # must do manually?
        self.log.log_data("begin takeoff procedure")
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
            if current_vertical_speed < takeoff_speed-2:
                throttle_val + 0.01
            if current_vertical_speed > takeof_speed+2:
                throttle_val - 0.01
            #''
            # update altitude
            temp_current_alt = self.sen.current_altitude
        '''
    # - modes end -
    
    def check_ready(self):
        check_pass = True
        # armed check
        self.sen.get_data()
        if self.sen.current_armed and self.con.require_disarm:
            check_pass = False
            self.log.log_data("check_ready - error: craft armed")
        # location check
        if self.con.location == "dem": pass
        else:
            check_pass = False
            self.log.log_data("check_ready - error: location not recognized")
        # return
        return check_pass

    # setup motor code stuff
    def setup(self):
        # inital check before script will accept control of craft.
        if self.check_ready() is False:
            self.log.log_data("move class - error: check_ready() failed")
            return False
        ## self.rc_reset_all()
        self.change_mode_guided()
        self.log.log_data("move class - setup complete")
        return True


#
# Testing class
# ------------------------------------------------------
#
class Testing:
    sen = Sensors()
    log = Logging()
    mov = Move()
    con = Config()

    def __init__(self):
        pass

    # test file operations
    def test_filelog(self):
        self.log.log_data("testing class - test_filelog() begin")
        log.log_data_custom("test_log", "test log data. \
            test_log_empty should be empty")
        log.log_data_custom("test_log_empty", "test log.  if here then FAIL")
        log.clear_log("test_log_empty")
        self.log_data("test_filelog - note: check for 'test_log' and '_empty'")
        self.log.log_data("testing class - test_filelog() complete")

    # test sensor class
    def test_sensors(self):
        self.log.log_data("testing class - test_sensors() begin")
        self.log.log_data("")
        for num in range(3):
            self.sen.get_data()
            self.sen.log_data()
            time.sleep(1)
        self.log.log_data("testing class - test_senssors() complete")

    # test arm and disarm - optional disarm (not tested)
    def test_arm(self):
        # arm
        self.log.log_data("testing class - test_arm() begin")
        self.log.log_data("testing class - arming craft")
        self.mov.arm_craft()
        if self.con.test_disarm is True:
            # print info user action
            self.log.log_data("testing class - note: flex throttle a bit to prevent auto disarm")
            self.log.log_data("testing class - disarming in 20 seconds")
            time.sleep(16)
            self.log.log_data("testing class - disarming in 4 seconds")
            time.sleep(4)
            # disarm
            self.log.log_data("testing class - disarming craft")
            self.mov.disarm_craft()
            self.log.log_data("testing class - test_arm() complete")

    # test takeoff
    def test_takeoff(self):
        self.log.log_data("test_takeoff - begin")
        if self.con.include_takeoff:
            # takeoff
            self.log.log_data("test_takeoff - taking off")
            ## self.mov.change_mode_takeoff()
        else:
            self.log.log_data("test_takeoff - skip takeoff")
        # land
        self.log.log_data("test_takeoff - landing")
        self.mov.change_mode_landing()
        self.mov.wait_for_land()
        self.log.log_data("test_takeoff - complete")

    # test waypoints
    def test_waypoints(self):
        self.log.log_data("test_waypoints - begin")
        if self.con.include_takeoff:
            pass
            ## self.mov.change_mode_takeoff()
        else:
            self.log.log_data("test_waypoints - note: ensure craft is flying already")
            self.log.log_data("test_waypoints - beggining in 6 seconds")
            time.sleep(6)
        # move to waypoint 1
        self.log.log_data("test_waypoints - move to waypoint one")
        if self.con.location == "dem":
            self.log.log_data("test_waypoints - demonte " + str(self.con.wp_1_index))
            self.mov.set_waypoint(self.con.loc_dem[self.con.wp_1_index], self.con.testing_altitude)
            self.mov.wait_waypoint_complete()
        # wp 2
        self.log.log_data("test_waypoints - move to waypoint two")
        if self.con.location == "dem":
            self.log.log_data("test_waypoints - demonte " + str(self.con.wp_2_index))
            self.mov.set_waypoint(self.con.loc_dem[self.con.wp_2_index], self.con.testing_altitude)
            self.mov.wait_waypoint_complete()
        # rtl
        if self.con.return_after:
            self.log.log_data("test_waypoints - rtl and land")
            self.mov.change_mode_rtl()
            self.mov.wait_waypoint_complete()
        self.log.log_data("test_waypoints - complete")


#
# Mission Class
# ------------------------------------------------------
# setup for tests of minor classes (Logging, Sensors, Move)
# note:
#     [start, launch, eject, recover, land]
#
class Mission:
    mov = Move()
    sen = Sensors()
    log = Logging()
    test = Testing()
    con = Config()

    # state variables
    # [start, launch, eject, recover, land]
    start_time = [0, 0, 0, 0, 0]
    start_pos = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    start_alt = [0, 0, 0, 0, 0]
    landed = False
    mission_begin = False
    mission_complete = False

    # init
    def __init__(self):
        if self.con.verbose:
            self.log.log_data("mission class - online")

    # reset all variables
    def reset_values(self):
        # start, launch, eject, recover, land
        self.start_time = [0, 0, 0, 0, 0]
        self.start_pos = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.start_alt = [0, 0, 0, 0, 0]
        # state variables
        self.landed = False
        self.mission_begin = False
        self.mission_complete = False

    # setup at start
    def setup(self):
        self.reset_values()
        # log starting data
        self.sen.get_data()  # update
        self.start_time[0] = self.sen.current_time
        self.start_pos[0] = [self.sen.current_lat,  self.sen.current_lng]
        self.start_alt = self.sen.current_altitude
        if self.con.verbose:
            self.log.log_data("mission class - setup complete")

    # Mission Alpha 01 - ARLISS main mission run function
    def ma_01(self):
        pass

    # run mission
    def run_mission(self):
        self.log.log_data("mission class - running mission")
        if (self.con.mission_mode == "0"):  # no mission
            self.log.log_data("mission class - no mission to run")
            self.log.log_data("mission class - exiting")
            return True
        elif (self.con.mission_mode == "t1"):  # test arm / disarm
            self.test.test_arm()
        elif (self.con.mission_mode == "t2"):  # test takeoff / landing
            self.test.test_takeoff()
        elif (self.con.mission_mode == "t3"):  # test waypoints
            self.test.test_waypoints()
        elif (self.con.mission_mode == "ma-01"):
            self.log.log_data("mission class - starting MissionAlpha-01")
            self.ma_01()
            self.log.log_data("mission class - mission complete")
        else:
            self.log.log_data("mission class - error: mission_mode unknown in run_mission()")
            self.log.log_data("mission class - run_mission() failed")
            return False  # exit
        self.log.log_data("mission class - run_mission() complete")
        return True

    def autorun(self):
        if self.mov.setup() is True:
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
    test = Testing()
    test.test_filelog()  # option
    test.test_sensors()  # option
    log.log_data("run_test complete")
else:
    # start script
    log.log_data("script online")
    log.log_data("-------------")
    mission = Mission()
    mission.autorun()
    log.log_data("---------------")
    log.log_data("script complete")
