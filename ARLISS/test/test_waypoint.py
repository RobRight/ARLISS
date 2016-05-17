import os
import time
import clr
clr.AddReference("MissionPlanner")
import MissionPlanner
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp
clr.AddReference("MAVLink") # includes the Utilities class
import MAVLink

loc = [39.4162037, -119.7361708, 40]

print("code begin")
#http://www.diydrones.com/forum/topics/mission-planner-python-script?commentId=705844%3AComment%3A1306487
new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,loc[0])		# set waypoint latitude
MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,loc[1])		# set waypoint longitude
MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,loc[2])		# set waypoint altitude
MAV.setGuidedModeWP(new_wp)
print("waypoint set")

# test waypoint distance for complete
#print("at waypoint")
