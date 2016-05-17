

import os
import time
import clr
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink") # includes the Utilities class
import MAVLink


class Move:

    # RC input [pin, min, max]
    rc_throttle = [3, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX')]
    rc_pitch = [2, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX')] # BACKWARDS
    rc_roll = [1, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX')]
    rc_yaw = [4, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX')]

    def __init__(self):
        # craft should be disarmed
        # change_mode_loiter()
        # zero out RC throttle
        # Script.SendRC(rc_throttle, 1000, True)
        pass

    def rc_value_map(self, chan, val): # val: 0 to 1
        min_val = chan[1]
        max_val = chan[2]
        return (max_val - min_val)*val + min_val

    # call to engage motor at 0 to 1 speed
    def rc_set_value(self, chan, in_val):
        if in_val > 1:
            print("error: rc_set_value greater than one.")
            return False
        val = self.rc_value_map(chan, in_val)
        Script.SendRC(chan[0], val, True)
        return True

    # set value to all four rc channels
    def rc_set_all(self, in_t, in_p, in_r, in_y):
        if in_t != -1: rc_set_value(rc_throttle, in_t)
        if in_p != -1: rc_set_value(rc_pitch, in_p)
        if in_r != -1: rc_set_value(rc_roll, in_r)
        if in_y != -1: rc_set_value(rc_yaw, in_y)

    # reset all rc value inputs to idle
    def rc_reset_all(self):
        # throttle min, pitch half, roll half, yaw half
        self.rc_set_all(self.rc_value_map(self.rc_throttle, 0), \
            self.rc_value_map(self.rc_pitch, 0.5), \
            self.rc_value_map(self.rc_roll, 0.5), \
            self.rc_value_map(self.rc_yaw, 0.5))

    def change_mode_loiter(self):
        Script.ChangeMode('LOITER')
        Script.WaitFor('LOITER', 5000) # whats this do?

    def change_mode_rtl(self):
        Script.ChangeMode('RTL')
        Script.WaitFor('RTL', 5000)

    def change_mode_landing(self):
        # http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_land
        # MAV_CMD_NAV_LAND(0,0,0,0,l,l) - NODE: do not know if this command works
        Script.ChangeMode('LAND') # set mode to LAND - http://copter.ardupilot.com/wiki/land-mode/
        # test for landed
        Script.WaitFor('LAND', 5000)

    def set_waypoint(self,loc): # add a wait for waypoint complete?? -----------------
        #http://www.diydrones.com/forum/topics/mission-planner-python-script?commentId=705844%3AComment%3A1306487
        new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
        MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,loc[0])		# set waypoint latitude
        MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,loc[1])		# set waypoint longitude
        MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,loc[2])		# set waypoint altitude
        MAV.setGuidedModeWP(new_wp)

    def arm_craft(self):
        print("arming motors")
        self.change_mode_loiter()
        self.rc_reset_all()
        self.rc_set_all(0,0,1,1)
        Script.WaitFor('ARMING MOTORS',15000)
        self.rc_reset_all()
        print("MOTORS ARMED")
        self.armed = True
        return True

    def disarm_craft(self):
        print("disarming motors")
        self.self.change_mode_loiter()
        self.rc_reset_all()
        self.rc_set_all(0,0,0,0)
        Script.WaitFor('DISARMING MOTORS',15000)
        self.rc_reset_all()
        print("MOTORS DISARMED")
        self.armed = False
        return True

def autostart():
    # run code here
    mov = Move()

    saved_wp_home = [39.415812, -119.734899, 60]
    saved_wp_1 = [39.416616, -119.736916, 60]

    # test mission 01
    #  --------------------------

    # test rc control
    #mov.rc_set_value()

    # test mission 10
    #  --------------------------

    # test waypoint navigation
    mov.set_waypoint(saved_wp_1)
    # test mode change (rtl)
    mov.change_mode_rtl()

    # test mission 11
    #  --------------------------

    # test mode landing
    #mov.change_mode_landing()

autostart()
