import numpy
import Adafruit_PCA9685
import rover_socket

class SharedVariables():
    def __init__(self):
        self.OBS = Obstacle()
        self.VI = Vision()
        self.LI = Lidar()
        self.MAP = MapPlotting()
        self.CAL = Calibration()
        self.CC = CarControl()

class Obstacle:
    def __init__(self):
        self.local_obstacle_x = numpy.array([0, 1, 2])
        self.local_obstacle_y = numpy.array([0, 1, 2])
        self.global_obstacle_x = numpy.array([])
        self.global_obstacle_y = numpy.array([])
        self.global_obstacle = numpy.array([])
        self.global_obstacle_buffer = list()

class Vision:
    def __init__(self):
        self.vision_ip = "192.168.5.101"
        self.vision = None
        self.vision_thread = None
        self.vision_run = False
        self.reset_flag = False
        self.vision_x = 0
        self.vision_y = 0
        self.vision_theta = 0


        self.vision_angle_radian = 0
        self.vision_status = -1
        self.vision_data = [0, 0, 0, self.vision_status]


class Lidar:
    def __init__(self):
        self.lidar = None
        self.lidar_thread = None
        self.lidar_USB_port = ""
        self.lidar_run_flag = False
        self.lidar_connect = False
        self.lidar_state = list()
        self.lidar_minimum_radius = 450
        self.lidar_data = [[0,0,0],[1,1,1]]
        self.lidar_angle = [0]
        self.lidar_radius = [0]

class MapPlotting:
    def __init__(self):
        self.arrow_x = [0]
        self.arrow_y = [0]
        self.global_map = numpy.array([])

class Calibration:
    def __init__(self):
        self.calibrate_x = 0
        self.calibrate_y = 0
        self.calibrate_angle = 0
        self.calibrate_x_multi = 1
        self.calibrate_y_multi = 1
        self.calibrate_angle_multi = 1
        self.calibrate_difference_between_lidar_and_vision = 130
        self.temp_calibrate_difference_between_lidar_and_vision = 130
        self.calibration_run = False

class CarControl:
    def __init__(self):
        self.car_control = Adafruit_PCA9685.PCA9685()
        self.car_control.set_pwm_freq(60)
        self.car_control_server_run = True
        self.car_control_receive = None
        self.car_control_previous_receive = None
        self.car_control_state = 'stop'
        self.car_control_forward_pwm = 410 # 403
        self.car_control_backward_pwm = 370  # 381
        self.car_control_stop_pwm = 400
        self.car_control_add_speed = 1