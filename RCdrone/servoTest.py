# Created by: Joe Sanford
# Modified by: William Gregory
# Last Date: 31 October 2015
# Thank you Joe!

# Description: ArduPlane servo test

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
#-------------------------------------------------------------

import sys
import math
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp


# Variables
#-------------------------------------------------------------

enable = True


# Functions
#-------------------------------------------------------------

def testServo(val, pos):
    if pos > 2000 or pos < 1000: return
    Script.SendRC(val, pos, True)


# Autostart
#-------------------------------------------------------------

def autostart():
    while enable:
        testServo(4, 1000)
        Script.Sleep(300)
        testServo(4, 1200)
        Script.Sleep(300)
    
autostart();
