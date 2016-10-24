#ARLISS - main.py - information

Using Python 2.7 and Mission Planner <br>
Note: This project is not complete


### Classes:
- Config: settings for mission and specific functions
- State: global state variables
- Logging: console and file output manager
- Sensors: craft sensor manager
- *Craft: modes, navigation, etc.
- *Rocket: rocket specific functions
- Testing: testing functions
- *Mission: mission manager.  main class

*important

### Outline of Mission:
1. In the rocket fairing on the ground
	* Idle
	* Low power
2. Rocket launch
3. Rocket payload release
	* Craft chute deploys
4. Craft recovery
	* Once at a set altitude, chute is cut and motors power on.
5. Navigation
	* Controlled decent
	* Horizontal travel towards target
6. Landing
	* Mission Complete

### Mission Setup:
1. Setup Mission Planner
	1. metric units
2. Setup Craft Initial
	1. set near rocket launch site and power on
	2. connect to Mission Planner
3. Setup Code Initial
	1. set Config:preset_config = "ma-01-init"
	2. run code
4. Setup Craft Final
	1. 
5. Setup Code Final
	1. set Config:preset_config = "ma-01"
	2. run code
6. Put craft in the can

### Code Sequence:
* Mission: autorun() <br>
	* Craft: setup()<br>
	* Mission: setup()
    	*Mission: reset_values()
    	* Craft: params_gps(False)
	* Mission: run_mission()
		* Config: config_default_ma01()
    	* Mission: ma_01()
      		* Rocket: wait_for_launch()
			* Rocket: wait_for_payload_release()
			* Rocket: recover()
			* Craft: navigation_manager(self.con.loc_brd_target, State.start_alt[0])
			* Craft: change_mode_landing()
			* Mission: end_mission()