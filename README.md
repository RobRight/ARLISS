# ARLISS
### ArduPilot Python 2.7 code

website: http://robright.github.io/ARLISS/

## Files:

 -------------------------------------------------------------------------------------- <br />
<b>ARLISS:</b> only offical flight code here <br />
<b>TestCode:</b> all other code here <br />
	- Other tests <br />
	- Drop code <br />



### main.py (main):

-- [ARLISS/main.py](ARLISS/main.py) --

### payload_drop.py (payload_drop):

[TestCode/drop_code/payload_drop.py] (TestCode/drop_code/payload_drop.py)

 -------------------------------------------------------------------------------------- <br />

## Useful Links:
<br />
general:<br />
	- ArduCopter: http://ardupilot.org/copter/index.html
	- Python and Mission Planner: http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html
<br />
parameters: http://ardupilot.org/copter/docs/parameters.html
flight modes: http://ardupilot.org/copter/docs/flight-modes.html
waypoints: http://www.diydrones.com/forum/topics/mission-planner-python-script
<br />
calculations:<br />
	- generate new location: http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	- direction to: http://www.movable-type.co.uk/scripts/latlong.html
<br />
logging:<br />
	- http://stackoverflow.com/questions/273192/how-to-check-if-a-directory-exists-and-create-it-if-necessary
	- http://askubuntu.com/questions/138922/path-to-user-desktop-using-python
<br />
## Python Reference:<br />
<br />
Information about ArduPilot (copter) python commands:<br />
--------------------------------------------------------------------------------------<br />
<br />
cs.x<br />
-----------<br />
- example: mode, armed, etc.<br />
- more info here: http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html<br />
<br />
Script.x<br />
-----------<br />
Script.ChangeParam(name,value) - change a parameter value<br />
Script.GetParam(name) - read a parameter value<br />
- copter parameters: http://ardupilot.org/copter/docs/parameters.html<br />
<br />
Script.ChangeMode(mode)<br />
- copter modes: http://ardupilot.org/copter/docs/flight-modes.html<br />
<br />
Script.SendRC(channel,pwm,sendnow) - set RC channel to value<br />
<br />
Script.WaitFor(string,timeout) - (not tested)<br />
<br />
MAV.x<br />
-----------<br />
MAV.doCommand(command);  - MAVLink Mission Command Messages (not tested)<br />
-command messages here: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/<br />
<br />
Script.Sleep(ms) - sleep time in milliseconds (I use time.sleep(s))<br />
 -------------------------------------------------------------------------------------- <br />
 <br />
 
## Code Example:

```python
# call at mission start
	def start():
		flight_mode = 0
		launch_pos = [sen.current_lat, sen.current_lon]
		launch_alt = sen.current_altitude
	
	# fly to waypoint
	def directed_flight():
		mov.set_waypoint(target_pos, alt)

# ------------
# mission code
# ------------
def main_run():
	start()
	# test until rocket launched
	while rocket_launched == false:
		if sen.current_altitude>start_alt+1000: rocket_launched = True
	print("rocket launched")
	
	# test until rocket ejected
	while ejected == false:
		if sen.current_altitude>max_alt: max_alt = sen.current_altitude
		if sen.current_altitude<max_alt+200: ejected = True # NOTE: maybe use accelerometer
	print("quad ejected")
	ejected_pos = [sen.curren_lat, sen.curren_lon]
	
	# wait until recovery start alt
	while sen.current_altitude>recovery_start_alt:
		# recover craft!!
		#if stable: recovered = True
		pass
	
	# navigate to target
	navigation_phase = True
	while navigation_phase:
		# set succsessive waypoints leading to target or just target waypoint??
		mov.set_waypoint(target_pos, landing_start_alt+1000)
		if sen.current_altitude < landing_start_alt:
			#if distance_to_target < landing_start_dist:
				navigation_phase = False

	if (mov.land_craft()):
		print("mission complete")
	
```


(posibly outdated)
 -------------------------------------------------------------------------------------- <br />
wayoints from code: http://diydrones.com/forum/topics/how-to-python-script-a-time-based-flight-plan <br />
MissionPlanner: http://planner.ardupilot.com/wiki/using-python-scripts-in-mission-planner/ <br />
ArduPilot: http://ardupilot.com/ <br />
Development Site: http://dev.ardupilot.com/ <br />
Plane Parameters: http://plane.ardupilot.com/wiki/arduplane-parameters/ <br />
MAVLink Common Message Set: https://pixhawk.ethz.ch/mavlink/ <br />
MAVLink Mission Command Messages: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/ <br />
<br />

GPS math stuff: Offset by distance: <br />
 - http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters <br />
 - http://gis.stackexchange.com/questions/5821/calculating-lat-lng-x-miles-from-point <br />
<br />

SITL: http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html <br />
 - SITL on Linux: http://dev.ardupilot.com/wiki/setting-up-sitl-on-linux/ <br />
 - SITL on Window in Linux VM: http://dev.ardupilot.com/wiki/setting-up-sitl-on-windows/ <br />

 -------------------------------------------------------------------------------------- <br />
