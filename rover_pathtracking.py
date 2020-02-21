import numpy as np


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
