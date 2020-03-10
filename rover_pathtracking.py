import numpy as np


class StanleyController:
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        self.last_target_index = 0


    def update_state(self):
        self.SV.PT.steering_angle = np.clip(
            self.SV.PT.steering_angle,
            - self.SV.PT.max_steering_angle,
            self.SV.PT.max_steering_angle
        )
        self.SV.PT.current_x += self.SV.PT.velocity * np.cos(self.SV.PT.current_yaw) * self.SV.PT.interval/1000
        self.SV.PT.current_y += self.SV.PT.velocity * np.sin(self.SV.PT.current_yaw) * self.SV.PT.interval/1000
        self.SV.PT.current_yaw += self.SV.PT.velocity / self.SV.PT.distance_front_rear_wheel *\
            np.tan(self.SV.PT.steering_angle) * self.SV.PT.interval/1000
        # self.SV.PT.current_yaw = self.SV.PT.current_yaw % 2*np.pi # normalize yaw to 0~2 pi
        self.SV.PT.velocity += self.SV.PT.acceleration * self.SV.PT.interval/1000

    def next_target(self):
        self.last_target_index = self.SV.PT.target_index
        # Calculate car head position
        head_x = self.SV.PT.current_x + self.SV.PT.distance_front_rear_wheel*np.cos(self.SV.PT.current_yaw)
        head_y = self.SV.PT.current_y + self.SV.PT.distance_front_rear_wheel*np.sin(self.SV.PT.current_yaw)
        distance = np.hypot(abs(self.SV.CF.fitted_route_x - head_x), abs(self.SV.CF.fitted_route_y - head_y))
        self.SV.PT.target_distance = np.min(distance)
        self.SV.PT.target_index = np.where(distance == self.SV.PT.target_distance)
        # if len(self.SV.PT.target_index) > 1:
            # self.SV.PT.target_index = self.SV.PT.target_index[0]


        # Target index in the format of (([], ""), ([], ""), ...)
        self.SV.PT.target_index = self.SV.PT.target_index[0][0]
        # if self.last_target_index >= self.SV.PT.target_index and self.SV.PT.velocity > 0:
        #     self.SV.PT.target_index = self.last_target_index
        # elif self.last_target_index <= self.SV.PT.target_index and self.SV.PT.velocity < 0:
        #     self.SV.PT.target_index = self.last_target_index

        self.SV.PT.target_position = [self.SV.CF.fitted_route_x[self.SV.PT.target_index], self.SV.CF.fitted_route_y[self.SV.PT.target_index]]


        # Used for debug
        self.SV.PT.distance_x = abs(self.SV.CF.fitted_route_x - head_x)
        self.SV.PT.distance_y = abs(self.SV.CF.fitted_route_y - head_y)
        self.SV.PT.distance = np.hypot(self.SV.PT.distance_x, self.SV.PT.distance_y)
        self.SV.PT.distance_strange = distance
        self.SV.PT.head_position = [head_x, head_y]
        print([head_x, head_y])

    def calculate_command(self):
        self.next_target()
        self.SV.PT.theta_e = self.SV.CF.fitted_route_yaw[self.SV.PT.target_index] - np.radians(self.SV.PT.current_yaw)
        self.SV.PT.steering_angle = np.degrees(self.SV.PT.theta_e + np.arctan((self.SV.PT.control_gain*self.SV.PT.target_distance)/self.SV.PT.velocity))


class CarModel:
    def __init__(self, SharedVariable):
        self.SV = SharedVariable
        self.distance_front_rear_wheel = 36 # cm
        self.distance_left_right_wheel = 34
        self.wheel_length = 18
        self.wheel_width = 7
        self.car_length = 55
        self.car_width = 40

    def plot_car(self, x=0, y=0, angle=0.0, steer=0.0):
        outline_x = np.array([
            self.car_length/(-2)*np.cos(angle) + x,
            self.car_length/(-2)*np.cos(angle) + x,
            self.car_length/(2)*np.cos(angle) + x,
            self.car_length/(2)*np.cos(angle) + x
        ])
        outline_y = np.array([
            self.car_width/(-2)*np.sin(angle) + y,
            self.car_width/(2)*np.sin(angle) + y,
            self.car_width/(-2)*np.sin(angle) + y,
            self.car_width/(2)*np.sin(angle) + y
        ])
        rear_rw_x = np.array([
            self.wheel_length/(2) + self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
            self.wheel_length/(-2) + self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
        ])
        rear_lw_x = np.array([
            self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
            self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
        ])
        rear_rw_y = np.array([
            self.distance_left_right_wheel/(-2)*np.sin(angle) + y,
            self.distance_left_right_wheel/(-2)*np.sin(angle) + y,
        ])
        rear_lw_y = np.array([
            self.distance_left_right_wheel/(2)*np.sin(angle) + y,
            self.distance_left_right_wheel/(2)*np.sin(angle) + y,
        ])
        front_rw_x = np.array([
            self.wheel_length/(-2)*np.cos(steer) + self.distance_front_rear_wheel/(2)*np.cos(angle) + x,
            self.wheel_length/(2)*np.cos(steer) + self.distance_front_rear_wheel/(2)*np.cos(angle) + x,
        ])
        front_lw_x = np.array([
            self.wheel_length/(-2)*np.cos(steer) + self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
            self.wheel_length/(2)*np.cos(steer) + self.distance_front_rear_wheel/(-2)*np.cos(angle) + x,
        ])
