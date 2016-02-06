import sys
import math
import clr
import time
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

import mavutil

target_location = [39.40456, -119.761292] # lat lon of drop target

# create and set new waypoint
def setNewWaypoint(wp_lat,wp_lng,wp_alt):
	#new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
	#MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,wp_lat)		# set waypoint latitude
	#MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,wp_lng)		# set waypoint longitude
	#MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,wp_alt)		# set waypoint altitude
	#MAV.setGuidedModeWP(new_wp)											# fly to the new 
	MAV.doCommand(MAVLink.MAV_CMD_NAV_WAYPOINT, 0, 2, 0, 0, wp_lat, wp_lng, wp_alt)

setNewWaypoint(target_location[0], target_location[1], 100)