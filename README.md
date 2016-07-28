# ARLISS

website: http://robright.github.io/ARLISS/

## Directory Outline:
-------------------------------------------------------------------------------------- <br />
<b>ARLISS:</b> only offical flight code here <br />
	* main.py: mission code<br />
	* info<br />
	* test<br />
<b>other:</b> generic test code <br />
	* Other tests <br />
	* Drop code <br />
### main.py (main):
-- [ARLISS/main.py](ARLISS/main.py) --
### payload_drop.py (payload_drop):
[other/drop_code/payload_drop.py] (other/drop_code/payload_drop.py)<br />
<br />
## Code Example:
-------------------------------------------------------------------------------------- <br />
```python
def ma_01(self):
	self.log.log_data("MA_01 begin")
	# setup mission
	self.setup_mission()
	# wait for rocket launch
	self.roc.wait_for_launch()
	# wait for payload release
	self.roc.wait_for_payload_release()
	# recovery phase
	self.roc.recover()
	# navigation phase
	self.cra.navigation_manager(self.con.loc_brd_target, log_alt[0])
	# landing phase
	self.cra.change_mode_landing()
	# mission complete
	self.end_mission()
	self.log.log_data("MA_01 complete")
```
<br />
## Useful Links:
-------------------------------------------------------------------------------------- <br />
general:<br />
	* ArduCopter: http://ardupilot.org/copter/index.html<br />
	* Python and Mission Planner: http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html<br />
<br />
parameters: http://ardupilot.org/copter/docs/parameters.html<br />
flight modes: http://ardupilot.org/copter/docs/flight-modes.html<br />
waypoints: http://www.diydrones.com/forum/topics/mission-planner-python-script<br />
<br />
calculations:<br />
	* generate new location: http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters<br />
	* direction to: http://www.movable-type.co.uk/scripts/latlong.html<br />
<br />
logging:<br />
	* http://stackoverflow.com/questions/273192/how-to-check-if-a-directory-exists-and-create-it-if-necessary<br />
	* http://askubuntu.com/questions/138922/path-to-user-desktop-using-python<br />
<br />
## Python Reference:
Information about ArduPilot (copter) python commands:<br />
--------------------------------------------------------------------------------------<br />
cs.x<br />
-----------<br />
example: mode, armed, etc.<br />
more info here: http://ardupilot.org/planner/docs/using-python-scripts-in-mission-planner.html<br />
<br />
Script.x<br />
-----------<br />
Script.ChangeParam(name,value) - change a parameter value<br />
Script.GetParam(name) - read a parameter value<br />
copter parameters: http://ardupilot.org/copter/docs/parameters.html<br />
Script.ChangeMode(mode)<br />
copter modes: http://ardupilot.org/copter/docs/flight-modes.html<br />
Script.SendRC(channel,pwm,sendnow) - set RC channel to value<br />
Script.WaitFor(string,timeout) - (not tested)<br />
<br />
MAV.x<br />
-----------<br />
MAV.doCommand(command);  - MAVLink Mission Command Messages (not tested)<br />
command messages here: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/<br />
Script.Sleep(ms) - sleep time in milliseconds (I use time.sleep(s))<br />
<br />
## More links:
(posibly outdated)<br />
-------------------------------------------------------------------------------------- <br />
wayoints from code: http://diydrones.com/forum/topics/how-to-python-script-a-time-based-flight-plan <br />
MissionPlanner: http://planner.ardupilot.com/wiki/using-python-scripts-in-mission-planner/ <br />
ArduPilot: http://ardupilot.com/ <br />
Development Site: http://dev.ardupilot.com/ <br />
Plane Parameters: http://plane.ardupilot.com/wiki/arduplane-parameters/ <br />
MAVLink Common Message Set: https://pixhawk.ethz.ch/mavlink/ <br />
MAVLink Mission Command Messages: http://plane.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/ <br />
GPS math stuff: Offset by distance: <br />
 * http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters <br />
 * http://gis.stackexchange.com/questions/5821/calculating-lat-lng-x-miles-from-point <br />
SITL: http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html <br />
 * SITL on Linux: http://dev.ardupilot.com/wiki/setting-up-sitl-on-linux/ <br />
 * SITL on Window in Linux VM: http://dev.ardupilot.com/wiki/setting-up-sitl-on-windows/ <br />
