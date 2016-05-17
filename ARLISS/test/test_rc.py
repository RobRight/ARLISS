'''

Test RC input control
- raw channel control
- arm/disarm
- takeoff
Mode control
- land
- loiter
- rtl

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

'''
RC pin, set by user from hardware configuration
RC min/max value for each channel, set at run time from flight controller configuration
RC input [pin, min, max]
Note: Check direction is correct and different radio configurations
Tested: True
Signed: WG 5/8/16
'''
rc_throttle = [3, Script.GetParam('RC3_MIN'), Script.GetParam('RC3_MAX'), 'throttle']
rc_pitch = [2, Script.GetParam('RC4_MIN'), Script.GetParam('RC4_MAX'), 'pitch']
rc_roll = [1, Script.GetParam('RC5_MIN'), Script.GetParam('RC5_MAX'), 'roll']
rc_yaw = [4, Script.GetParam('RC6_MIN'), Script.GetParam('RC6_MAX'), 'yaw']


'''
Inputs: int(0:1) - ratio of min/max.  0 for off, 1 for full
Outputs: RC value mapped using input and min/max for that channel
Description: Returns a mapped value using the RC channels min/max value and an input value between 0 and 1
Notes: If the input value is greater than 1 or less than 0 it will fail.
Tested: True
Signed: WG 5/8/16
'''
def rc_value_map(chan, val):
    if val > 1 or val < 0:
        print("error: rc_set_value greater than one. chan:" + str(chan[3]) +" val:" + str(in_val))
        return False
    min_val = chan[1]
    max_val = chan[2]
    return (max_val - min_val)*val + min_val


'''
Inputs: channel and speed value (0:1)
Description: sets that RC channel to mapped input value
Tested: True
Signed: WG 5/8/16
'''
def rc_set_value(chan, val):
    mval = rc_value_map(chan, val)
    Script.SendRC(chan[0], mval, True)
    return True


'''
Inputs: RC value (0:1) (x4)
Description: sets all four of the standard RC inputs (throttle, pitch, roll, yaw)
Tested: True
Signed: WG 5/8/16
'''
def rc_set_all(in_t, in_p, in_r, in_y):
    if in_t != -1: rc_set_value(rc_throttle, in_t)
    if in_p != -1: rc_set_value(rc_pitch, in_p)
    if in_r != -1: rc_set_value(rc_roll, in_r)
    if in_y != -1: rc_set_value(rc_yaw, in_y)
    return True


'''
Description: reset all RC channels to default
Tested: True
Signed: WG 5/8/16
'''
def rc_reset_all():
    print("reset all RC")
    # throttle min, pitch half, roll half, yaw half
    rc_set_all(0, 0.5, 0.5, 0.5)
    return True


'''
Description: reset all RC channels to default
Tested: False
'''
def change_mode_loiter():
    print("change mode to loiter")
    Script.ChangeMode('LOITER')
    Script.WaitFor('LOITER', 5000)
    return True


'''
Description: arm craft for flight
Note: motors will spin once armed, check for failsafes and mode
Tested: True
Signed: WG 5/8/16
'''
def arm_craft():
    print("arming motors")
    #change_mode_loiter()
    rc_reset_all()
    rc_set_all(0,0,1.0,1.0)
    Script.WaitFor('ARMING MOTORS',15000)
    rc_reset_all()
    print("arm complete")
    #armed = True
    return True


'''
Description: reset all RC channels to default
Tested: False
'''
def disarm_craft():
    print("disarming motors")
    #change_mode_loiter()
    rc_reset_all()
    rc_set_all(0,0,0,0)
    Script.WaitFor('DISARMING MOTORS',15000)
    rc_reset_all()
    print("disarm complete")
    #armed = False
    return True


'''
Description: takeoff the craft from the ground and loiter at set alitude
Tested: False, needs more work
'''
def takeoff_from_ground():
    '''
    # - settings start -
    desired_takeoff_alt = 6 # desired increased altitude
    takeoff_speed = 2 # desired vertical speed
    takeoff_throttle = 0.6
    # - settings end -
    print("begin takeoff")
    takeoff_alt = self.sen.current_altitude
    arm_craft()
    rc_set_value(rc_throttle, takeoff_speed)
    # maintain until at set altitude, adjust takeoff speed
    while temp_current_alt < takeoff_alt + desired_takeoff_alt:
        temp_current_alt = self.sen.current_altitude
        temp_vertical_speed = self.sen.vertical_speed
        rc_set_value(rc_throttle, throttle_val)
        # monitor vertical speed
        if current_verticle_speed < takeoff_speed-2:
            throttle_val + 0.01
        if current_verticle_speed > takeof_speed+2:
            throttle_val - 0.01
    '''
    return True


# --------------------------------------------------------
# --------------------------------------------------------


test_type = -1;
print("test begin: " + str(test_type))

'''
test throttle
setup: arm craft and hold in position
test: the throttle will go up to half and then back to zero
result: successful 5/8/16
'''
if test_type == 1:
    print("rc throttle to 0.5 (half)")
    rc_set_value(rc_throttle, 0.5)
    time.sleep(1)
    print("rc throttle to 0 (off)")
    rc_set_value(rc_throttle, 0)

# ----------------------------
'''
test arm/disarm
setup: power on craft
test: the craft will arm, then disarm shortly after
result: successful 5/8/16
'''
elif test_type == 2:
    arm_craft()
    # after armed power up motors and lower again to keep from auto disarming, then code will disarm
    time.sleep(10)
    disarm_craft()
# ----------------------------
'''
test takeoff
setup: power on craft
test: the craft takeoff and enter loiter mode
result:
'''
elif test_type == 3:
    takeoff()
# ----------------------------
else:
    print("no test")
# ----------------------------
if(test_type != -1) print("test complete")
