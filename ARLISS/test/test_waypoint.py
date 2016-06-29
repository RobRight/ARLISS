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
# Sensors Class
# ------------------------------------------------------
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


class Move:
    sen = Sensors()

    verbose = True
    waypoint_tolerance = 2

    def __init__(self):
        pass

	# - waypoints begin -
	# passing all (loc,lng,alt) values will send this waypoint to craft, begining immediatly.
    def set_waypoint(self, loc_lat, loc_lng, loc_alt):
    #http://www.diydrones.com/forum/topics/mission-planner-python-script?commentId=705844%3AComment%3A1306487
    new_wp = MissionPlanner.Utilities.Locationwp()						# create waypoint object
    MissionPlanner.Utilities.Locationwp.lat.SetValue(new_wp,loc[0])		# set waypoint latitude
    MissionPlanner.Utilities.Locationwp.lng.SetValue(new_wp,loc[1])		# set waypoint longitude
    MissionPlanner.Utilities.Locationwp.alt.SetValue(new_wp,loc[2])		# set waypoint altitude
    MAV.setGuidedModeWP(new_wp)
    if (self.verbose) print("waypoint set: lat:" + str(loc[0]) + " lng:" + str(loc[1]) + " alt:" + str(loc[2]))
    # can check waypoint distance for complete

	# returns True if waypoint minimum distance is met
    def waypoint_complete(self):
        if sen.current_distance < waypoint_tolerance: return True
        return False

	# waits for waypoint_complete() to finish
    def wait_waypoint_complete(self):
        while(self.waypoint_complete()): pass
	# - waypoints end -

# autostart
def autostart():
    print("code begin")
    mov = Move()

# Mission functions
def path_test():
	print("testing waypoints")
	Script.ChangeMode("Guided")
	if (self.verbose) print("guided mode")
    if (self.verbose) print("begin path")
    loc = [39.417289, -119.736879]
    loc2 = [39.416095, -119.736707]
    mov = Move()
    # one
	if (self.verbose) print("location one (1)")
    mov.set_waypoint(*loc, 100)
    wait_waypoint_complete()
    # two
	if (self.verbose) print("location two (2)")
    mov.set_waypoint(*loc, 50)
    wait_waypoint_complete()
	print("waypoint test complete")

# runs at start
autostart()
