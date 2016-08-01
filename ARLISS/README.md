ARLISS - main.py - information <br />
<br />
Using Python 2.7 and Mission Planner
<br />
###Classes:
- Config: settings for mission and specific functions
- State: global state variables
- Logging: console and file output manager
- Sensors: craft sensor manager
- Craft: modes, navigation, etc.
- Rocket: rocket specific functions
- Testing: testing functions
- Mission: mission manager.  main class
<br />
<br />
###Outline of Mission:
1. In the rocket fairing on the ground
	* Idle
	* Low power
2. Rocket launch
3. Rocket payload release
	* Momentary free fall
4. Craft recovery
	* Orientation control - stable flight
5. Navigation
	* Controlled decent
	* Horizontal travel towards target
6. Landing
	* Mission Complete