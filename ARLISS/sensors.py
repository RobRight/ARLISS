#
# Sensors Class
# ------------------------------------------------------
# Manages the crafts flight variables.
# Call get_data() to update values.
#
class Sensors:
    log = Logging()

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
    # note: check that all variables are current, used, and all that are needed are here.
    # Tested: no
    def get_data(self):
        # self.log.log_data("logging class - got new sensor data")

        self.current_time = time.time()
        self.current_mode = cs.mode  # NOTE
        self.current_distance = cs.wp_dist  # NOTE
        self.current_waypoint = cs.wpno
        self.current_altitude = cs.alt  # NOTE
        self.current_vertical_speed = cs.verticalspeed  # NOTE
        self.current_ground_speed = cs.groundspeed  # NOTE
        self.current_ground_course = cs.groundcourse
        self.current_wind_direction = cs.wind_dir
        self.current_wind_speed = cs.wind_vel
        self.current_roll = cs.roll  # NOTE
        self.current_pitch = cs.pitch  # NOTE
        self.current_yaw = cs.yaw
        self.current_lat = cs.lat
        self.current_lng = cs.lng
        self.current_gps_stat = cs.gpsstatus
        self.current_gps_count = cs.satcount
        self.current_battery_voltage = cs.battery_voltage
        self.current_armed = cs.armed  # NOTE
        self.current_altitude_error = cs.alt_error

        self.current_accel = []
        self.current_accel.append(cs.ax)
        self.current_accel.append(cs.ay)
        self.current_accel.append(cs.az)

        self.current_gyro = []
        self.current_gyro.append(cs.gx)
        self.current_gyro.append(cs.gy)
        self.current_gyro.append(cs.gz)
    
    # log sensor data to 'log_data' file
    # note: could add more values
    # Tested: no
    def log_data(self):
        self.log.log_data("- logging sensor data -")
        self.log.log_data("current_time: %f" % self.current_time)
        self.log.log_data("current_mode: " + self.current_mode)
        self.log.log_data("current_altitude: " + str(self.current_altitude))
        self.log.log_data("current_ground_speed: " + str(self.current_ground_speed))
        self.log.log_data("current_gps_count: " + str(self.current_gps_count))
        self.log.log_data("current_armed: " + str(self.current_armed))
        self.log.log_data("")