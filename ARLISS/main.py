# ARLISS QUAD PYTHON CODE

#import sys
#import math
#import clr
#import time
clr.AddReference("MissionPlanner")
import MissionPlanner
#clr.AddReference("MissionPlanner.Utilities")
#from MissionPlanner.Utilities import Locationwp
#clr.AddReference("MAVLink") # includes the Utilities class
#import MAVLink




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


# get updated flight data
def getFlightData():
	global current_time, current_distance, current_altitude, current_groundspeed, current_groundcourse, current_waypoint, current_wind_direction, current_wind_speed
	current_time = time.time()
	current_distance = cs.wp_dist
	current_altitude = cs.alt
	current_groundspeed = cs.groundspeed
	current_groundcourse = cs.groundcourse
	current_waypoint = cs.wpno
	current_wind_direction = cs.wind_dir
	current_wind_speed = cs.wind_vel
	
# run at script start
def autostart():
	print("Hello ARLISS")
	
autostart();