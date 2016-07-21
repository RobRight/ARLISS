# 
# ARLISS Mission Script
# 
# 7/1/2016 to current
# http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html
#
# ToDo:
# - disable GPS before and during launch
# - detect launch (barometer, acclerameter)
# - detect rocket ejection (accelerometer, altitude, vertical speed)
# - test parameter manage function (GPS, Failsafes)
# - add parameter manage for PIDs
# - test arming in midflight
# 


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

# mission options:
# -----------------
# (0) no mission
# (t1) test arm and disarm - arms on ground, seconds later disarms.
# (t2) test takeoff and landing - goes up and lands at the same location.
# (t3) test waypoints - flys to a few waypoints, then rtl and land
# (t4) test recovery - check settings in config class
# (t5) test_navigation
# (ma-01) - mission_alpah - complete mission from idle, launch, recovery, navigation, and landling.

# location options:
# -----------------
# (dem) Demonte Ranch
# (brd) (NI) Black Rock Desert (NI) - not implemented

class Config:
    # - general settings -
    verbose = True
    mission_mode = "t4"
    location = "dem"
    run_test = False # sensor and file testing
    require_disarm = False
    disable_gps_on_start = False
    # navigation
    jump_distance = 100  # distance to jump
    jump_alt = 80  # verticle distance to jump
    # recovery
    recover_arm = False  # mission (true)
    wait_recov = True  # wait until recovered - NR
    # takeoff
    takeoff_throttle_val = 0.7
    default_takeoff_alt = 20  # m ??
    default_takeoff_speed = 2  # m/s ??
    # land
    desired_vert_speed = -0.2 # once vs here. landed
    # waypoints
    waypoint_tolerance = 5  # m ??
    ## custom_path = [0,1,2,3,4,5,6,7,8,9,10]
    # ---------------------------
    # - testing class -
    # test_arm function
    test_disarm = False  # not implemented
    # test_waypoints function
    include_takeoff_wp = True
    testing_altitude = 30  # units
    wp_1_index = 2  # test_waypoint loc 1
    wp_2_index = 5
    return_after = True
    # test takeoff
    include_takeoff_t = True
    hold_position_time = 10
    # test recovery
    test_recover_start_alt = 120
    takeoff_before_recover = True
    flyto_recover = True  # starting position
    recover_test_sleep = 3  # sleep to allow free fall
    fly_back_home = True  # fly back to pickup after test
    # - logging class -
    log_enable = True  # enable file logging
    print_enable = True  # enable console logging
    default_name = "log_file"  # file log filename prefix
    # - move class -
    # min distance from waypoint before moving on
    # rc pins
    rc_throttle_pin = 3
    rc_pitch_pin = 2
    rc_roll_pin = 1
    rc_yaw_pin = 4
    # esc pins
    esc_f_pin = 3
    esc_b_pin = 4
    esc_l_pin = 2
    esc_r_pin = 1
    # ---------------------------
    # locations: [latitude, longitude]
    # random
    loc_rand_unr = [39.550409, -119.809378]  # UNR
    loc_rand_tahoe = [39.221711, -119.928340]  # Lake Tahoe
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
            current_time = time.strftime("%H_%M_%S: ", time.localtime())
            f.write(current_time + in_data + "\n")
            f.close()
        if self.con.print_enable is True:
            print(in_data);
    
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


# Craft class
# ------------------------------------------------------
# waypoints, land, arm/disarm
# note: need loiter function; check channel mappings
#
class Craft:
    sen = Sensors()
    log = Logging()
    con = Config()

    # RC input [pin, min, max] - check direction
    rc_throttle = [con.rc_throttle_pin, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
    rc_pitch = [con.rc_pitch_pin, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')]  # B
    rc_roll = [con.rc_roll_pin, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX')]
    rc_yaw = [con.rc_yaw_pin, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX')]
    # ESC output [pin, min, max]
    # esc_f = [esc_f_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_b = [esc_b_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_l = [esc_l_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]
    # esc_r = [esc_r_pin, Script.GetParam('esc_min'), Script.GetParam('esc_max')]

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
        self.change_mode_stabilize()
        self.rc_reset_all()
        # left stick to bottom right.  throttle and yaw channels
        self.rc_set_all(0, -1, -1, 1)
        temp_armed = False
        while temp_armed is False:
            self.sen.get_data()
            temp_armed = self.sen.current_armed
        self.rc_reset_all()
        self.log.log_data("move class - motors armed")
        self.armed = True
        return True

    # disarm motors (low power mode)
    # pass: NA
    def disarm_craft(self):
        self.log.log_data("move class - disarming motors")
        self.change_mode_stabilize()
        self.rc_reset_all()
        # left stick bottom left. throttle and yaw channels
        self.rc_set_all(0, -1, -1, 0)
        temp_armed = True
        while temp_armed is True:
            self.sen.get_data()
            temp_armed = self.sen.current_armed
        self.rc_reset_all()
        self.rc_reset_all()
        self.log.log_data("move class - motors disarmed")
        self.armed = False
        return True
    # - rc end -

    # - waypoints begin -
    # returns the distance between two locations
    # input: start_loc: [lat, lon], end_loc: [lat, lon]
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

    #
    #
    # http://www.movable-type.co.uk/scripts/latlong.html
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
    # http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
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

    # passing all ([loc,lng],alt) values will send this waypoint to craft
    # input: [latitude, longitude], (int) altitude (units???)
    # pass: NA
    def set_waypoint(self, loc, loc_alt):
        # http://www.diydrones.com/forum/topics/mission-planner-python-script
        new_wp = MissionPlanner.Utilities.Locationwp()  # create waypoint object
        MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp, loc[0])  # set waypoint latitude
        MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp, loc[1])  # set waypoint longitude
        MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp, loc_alt)  # set waypoint altitude
        self.change_mode_guided()
        MAV.setGuidedModeWP(new_wp)  # begin waypoint
        # logs waypoint info
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
        return True

    # 
    # input: target: [lat, lon], target_alt: target altitude
    # note: add types like directional or decent
    def navigation_manager(self, target, target_alt):
        self.log.log_data("move class - navigation start")
        self.sen.get_data()
        total_dist = self.calc_distance([self.sen.current_lat, self.sen.current_lng], target)
        total_alt = math.fabs(self.sen.current_altitude - target_alt)
        nav_complete = False
        while nav_complete == False:
            self.sen.get_data()
            current_loc = [self.sen.current_lat, self.sen.current_lng]
            current_alt = self.sen.current_altitude
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
                self.log.log_data("move class - navigation: next waypoint")
                self.set_waypoint(temp_jump_loc, temp_alt)
                self.wait_waypoint_complete()
            else:
                # next jump is target
                self.log.log_data("move class - moving to final waypoint")
                self.set_waypoint(target, target_alt)
                self.wait_waypoint_complete()
                nav_complete = True
            
        self.log.log_data("move class - navigation complete")
    # - waypoints end -

    # - modes begin -
    # engage landing
    # do not return until landing complete or timeout
    # pass: yes
    def change_mode_landing(self):
        Script.ChangeMode('LAND')  # set mode to LAND
        # http://copter.ardupilot.com/wiki/land-mode/

    # waits for landing based on verticle speed
    # note: blocking function. could add timeout
    # pass: NA
    def wait_for_land(self):
        self.log.log_data("move class - waiting for land")
        time.sleep(2)
        landing = True
        while landing:
            self.sen.get_data()
            if self.sen.current_vertical_speed < 0.0 and self.sen.current_vertical_speed > self.con.desired_vert_speed:
                landing = False
        temp_armed = True
        while temp_armed:
            self.sen.get_data()
            temp_armed = self.sen.current_armed
        self.change_mode_guided()
        self.log.log_data("move class - landing complete")
        return True

    # change mode to loiter
    # info: hold position and altitude
    # pass: NA
    def change_mode_loiter(self):
        Script.ChangeMode('LOITER')
        # Script.WaitFor('LOITER', 5000)  # whats this do?

    def change_mode_stabilize(self):
        Script.ChangeMode('STABILIZE')

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
    # note: desired vs is 2
    # pass: NA - not ready
    def change_mode_takeoff(self):
        self.log.log_data("move class - begin takeoff")
        self.sen.get_data()
        temp_start_alt = self.sen.current_altitude
        # arm
        self.arm_craft()
        # enter loiter mode
        self.change_mode_stabilize()
        # engage motors; warmup then throttle at 80%
        self.rc_reset_all()
        self.rc_set_value(self.rc_throttle, self.con.takeoff_throttle_val)
        time.sleep(1)
        taking_off = True
        self.log.log_data("move class - going up")
        self.log.log_data("move class - waiting for altitude")
        # maintain until at set altitude
        while taking_off:
            self.sen.get_data()
            # monitor vertical speed
            if self.sen.current_vertical_speed < self.con.default_takeoff_speed:
                self.con.takeoff_throttle_val + 0.01
            if self.sen.current_vertical_speed > self.con.default_takeoff_speed+1:
                self.con.takeoff_throttle_val - 0.01
            # set throttle
            self.rc_set_value(self.rc_throttle, self.con.takeoff_throttle_val)
            # update altitude
            if self.sen.current_altitude > (temp_start_alt + self.con.default_takeoff_alt):
                taking_off = False
        self.change_mode_guided()
        self.set_waypoint([self.sen.current_lat, self.sen.current_lng], self.con.default_takeoff_alt)
        self.rc_reset_all()
        self.log.log_data("move class - takeoff complete")
    # - modes end -

    # - parameters -
    # gps param enable and disable (True to enable)
    def params_gps(self, in_en):
        if in_en:
            # enable gps
            self.change_mode_stabilize()
            Script.ChangeParam(AHRS_GPS_USE, 1)
        else:
            # disable gps
            Script.ChangeParam(AHRS_GPS_USE, 0)

    # params setup failsafes
    def params_failsafe_setup(self):
        Script.ChangeParam(FS_BATT_ENABLE, 0)  # 1:land if low battery, 0:disable (dont stop till you drop!)
        Script.ChangeParam(FS_GCS_ENABLE, 0)  # 0:disabled (local ground station?)
        Script.ChangeParam(FS_THR_ENABLE, 0)  # 0:disabled (no radio for mission)
        Script.ChangeParam(FS_EKF_ACTION, 1)  # 1:land
        Script.ChangeParam(FS_EKF_THRESH, 1.0)  # 1:relaxed (probably needed with rough flight)

    # default params
    def params_setup(self):
        Script.ChangeParam(FENCE_ENABLE, 0)  # 0:disabled
        self.params_failsafe_setup()
        if self.con.disable_gps_on_start:
            self.params_gps(False)
    # - parameters end - 

    def check_ready(self):
        check_pass = True
        # armed check
        self.sen.get_data()
        if self.sen.current_armed and self.con.require_disarm:
            check_pass = False
            self.log.log_data("check_ready - error: craft armed")
        # location check
        if self.con.location == "dem": pass # load values into loc variable
        else:
            check_pass = False
            self.log.log_data("check_ready - error: location not recognized")
        # gps lock
        if self.sen.current_gps_stat != 3:
            check_pass = False
            self.log.log_data("check_ready - error: gps fail")
        # return
        return check_pass

    # setup motor code stuff
    def setup(self):
        # inital check before script will accept control of craft.
        if self.check_ready() is False:
            self.log.log_data("move class - error: check_ready() failed")
            return False
        self.rc_reset_all()
        self.params_setup()
        self.change_mode_guided()
        self.log.log_data("move class - setup complete")
        return True


class Rocket:
    sen = Sensors()
    cra = Craft()
    log = Logging()
    con = Config()

    def __init__(self):
        pass

    def detect_launch(self):
        pass  # check acceleromiter or alitude

    def detect_ejection(self):
        pass  # check acceleromiter or alitude

    def wait_for_recover(self):
        self.log.log_data("mission class - wait for recovery")
        # check sensors
        # if level wait a few seconds
        # if still level then return
        self.log.log_data("mission class - craft recovered")

    def recover(self):
        # assumed terminal velocity: 35 to 40 m/s
        self.log.log_data("mission class - recovery start")
        if (self.con.recover_arm):
            self.cra.arm_craft()
        self.cra.change_mode_stabilize()
        self.cra.rc_set_value(self.cra.rc_throttle, 0.8)
        time.sleep(1)
        self.cra.change_mode_loiter()
        self.sen.get_data()
        self.cra.set_waypoint([self.sen.current_lat, self.sen.current_lng], (self.sen.current_altitude-10))
        if (self.con.wait_recov):
            ## testing only
            self.sen.get_data()
            current_vs = self.sen.current_vertical_speed
            while abs(current_vs) > 1.0:
                self.sen.get_data()
                current_vs = self.sen.current_vertical_speed
            ## self.wait_for_recover()
        else:
            time.sleep(4)
        self.log.log_data("mission_clas - recovery complete")


#
# Testing class
# ------------------------------------------------------
#
class Testing:
    sen = Sensors()
    log = Logging()
    cra = Craft()
    con = Config()
    rok = Rocket()

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

    # test waypoints
    def test_waypoints(self):
        self.log.log_data("test_waypoints - begin")
        if self.con.include_takeoff_wp:
            self.log.log_data("test_waypoints - taking off")
            self.cra.change_mode_takeoff()
        else:
            self.log.log_data("test_waypoints - note: ensure craft is flying already")
            self.log.log_data("test_waypoints - beggining in 6 seconds")
            time.sleep(6)
        # move to waypoint 1
        self.log.log_data("test_waypoints - move to waypoint one")
        if self.con.location == "dem":
            self.log.log_data("test_waypoints - demonte " + str(self.con.wp_1_index))
            self.cra.set_waypoint(self.con.loc_dem[self.con.wp_1_index], self.con.testing_altitude)
            self.cra.wait_waypoint_complete()
        # wp 2
        self.log.log_data("test_waypoints - move to waypoint two")
        if self.con.location == "dem":
            self.log.log_data("test_waypoints - demonte " + str(self.con.wp_2_index))
            self.cra.set_waypoint(self.con.loc_dem[self.con.wp_2_index], self.con.testing_altitude)
            self.cra.wait_waypoint_complete()
        # rtl
        if self.con.return_after:
            self.log.log_data("test_waypoints - rtl and land")
            self.cra.change_mode_rtl()
            self.cra.wait_waypoint_complete()
        self.log.log_data("test_waypoints - complete")

    # test recovery
    def test_recovery(self):
        self.log.log_data("test_recovery - begin")
        if (self.con.location != "dem"):  # location check
            self.log.log_data("test_recovery - location error")
            return False
        if (self.con.takeoff_before_recover):  # takeoff
            self.cra.change_mode_takeoff()
        if (self.con.flyto_recover):
            self.log.log_data("test_recovery - flying to start position")
            self.cra.set_waypoint(self.con.loc_dem[5], self.con.test_recover_start_alt)  # recovery location
            self.cra.wait_waypoint_complete()
        self.log.log_data("test_recovery - disableing craft")
        self.cra.change_mode_stabilize()
        self.cra.rc_set_value(self.cra.rc_throttle, 0.5)
        time.sleep(1)
        self.cra.rc_reset_all()  # cut throttle
        if (self.con.recover_arm):
            self.cra.disarm_craft()  # disarm
        time.sleep(self.con.recover_test_sleep)  # wait
        self.log.log_data("test_recovery - starting recovery")
        self.rok.recover()  # recover
        if (self.con.fly_back_home):
            self.log.log_data("test_recovery - flying back")
            self.cra.navigation_manager(self.con.loc_dem[0], 20)
            self.cra.change_mode_landing()

    # test navigation
    def test_navigation(self):
        self.log.log_data("test_navigation - begin")
        self.cra.navigation_manager(con.loc_rand_unr, 20)
        self.log.log_data("test_navigation - end")


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
        elif (self.con.mission_mode == "t4"):  # test recovery
            self.test.test_recovery()
        elif (self.con.mission_mode == "t5"):  # test navigation
            self.test.test_navigation()
        elif (self.con.mission_mode == "ma-01"):
            self.log.log_data("mission class - starting MissionAlpha-01")
            self.ma_01()
            self.log.log_data("mission class - mission complete")
        else:
            self.log.log_data("mission class - error: mission_mode unknown in run_mission()")
            self.log.log_data("mission class - run_mission() failed")
            return False  # exit
        self.log.log_data("mission class - mission complete")
        return True

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
