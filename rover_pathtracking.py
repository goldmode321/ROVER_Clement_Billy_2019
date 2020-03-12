import numpy as np


class StanleyController:
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        self.last_target_index = 0
        self.difference = None
        self.difference_x = None
        self.difference_y = None


    def update_state(self):
        self.SV.PT.current_x += self.SV.PT.velocity * np.cos(np.radians(self.SV.PT.current_yaw)) * self.SV.PT.interval/1000
        self.SV.PT.current_y += self.SV.PT.velocity * np.sin(np.radians(self.SV.PT.current_yaw)) * self.SV.PT.interval/1000
        self.SV.PT.current_yaw += np.degrees(self.SV.PT.velocity / self.SV.PT.distance_front_rear_wheel *\
            np.tan(self.SV.PT.real_steer_rad) * self.SV.PT.interval/1000)
        # self.SV.PT.current_yaw = self.SV.PT.current_yaw % 2*np.pi # normalize yaw to 0~2 pi
        self.SV.PT.velocity += self.SV.PT.acceleration * self.SV.PT.interval/1000

    def next_target(self):
        self.last_target_index = self.SV.PT.target_index
        # Calculate car head position
        head_x = self.SV.PT.current_x + self.SV.PT.distance_front_rear_wheel*np.cos(np.radians(self.SV.PT.current_yaw))
        head_y = self.SV.PT.current_y + self.SV.PT.distance_front_rear_wheel*np.sin(np.radians(self.SV.PT.current_yaw))
        self.difference_x = self.SV.CF.fitted_route_x - head_x
        self.difference_y = self.SV.CF.fitted_route_y - head_y
        self.difference = np.hypot(self.difference_x, self.difference_y)
        self.SV.PT.target_distance = np.min(self.difference)
        self.SV.PT.target_index = np.where(self.difference == self.SV.PT.target_distance)
        # if len(self.SV.PT.target_index) > 1:
            # self.SV.PT.target_index = self.SV.PT.target_index[0]


        # Target index in the format of (([], ""), ([], ""), ...)
        self.SV.PT.target_index = self.SV.PT.target_index[0][0]
        if self.last_target_index >= self.SV.PT.target_index:
            self.SV.PT.target_index = self.last_target_index
        self.SV.PT.target_position = [self.SV.CF.fitted_route_x[self.SV.PT.target_index], self.SV.CF.fitted_route_y[self.SV.PT.target_index]]



        front_axle_vec = [-np.cos(np.radians(self.SV.PT.current_yaw) + np.pi / 2),
                        - np.sin(np.radians(self.SV.PT.current_yaw) + np.pi / 2)]
        self.error_front_axle = np.dot(
            [self.difference_x[self.SV.PT.target_index], self.difference_y[self.SV.PT.target_index]], front_axle_vec)


    def calculateCommand(self):
        self.next_target()
        # self.SV.PT.theta_e = np.radians(self.SV.PT.current_yaw) - self.SV.CF.fitted_route_yaw_rad[self.SV.PT.target_index]
        self.SV.PT.theta_e = self.normalizeAngleRadian(np.radians(self.SV.PT.current_yaw) - self.SV.CF.fitted_route_yaw_rad[self.SV.PT.target_index])

        direction = self.findDirection()

        # self.SV.PT.steering_target_angle_rad = ((self.SV.PT.theta_e + \
        #     np.arctan((self.SV.PT.control_gain*self.SV.PT.target_distance)\
        #         /self.SV.PT.velocity)) * direction + np.radians(self.SV.PT.current_yaw))%(2*np.pi)

        self.SV.PT.steering_target_angle_rad = ((self.SV.PT.theta_e + \
            np.arctan2((self.SV.PT.control_gain*self.error_front_axle),\
                self.SV.PT.velocity)) * direction + np.radians(self.SV.PT.current_yaw))%(2*np.pi)

        self.SV.PT.steering_target_angle_deg = np.degrees(self.SV.PT.steering_target_angle_rad)



        normalized_target_angle = self.normalizeAngleDegree(self.SV.PT.steering_target_angle_deg)
        normalized_yaw = self.normalizeAngleDegree(self.SV.PT.current_yaw)
        self.SV.PT.steering_angle_deg = np.clip(
            normalized_target_angle,
            normalized_yaw - self.SV.PT.max_steering_angle,
            normalized_yaw + self.SV.PT.max_steering_angle
        )
        self.SV.PT.steering_angle_rad = np.radians(self.SV.PT.steering_angle_deg)
        self.SV.PT.real_steer_deg = self.SV.PT.steering_angle_deg - self.SV.PT.current_yaw
        self.SV.PT.real_steer_rad = np.radians(self.SV.PT.real_steer_deg)

            # self.SV.PT.steering_angle_rad = np.clip(
            #     self.SV.PT.steering_target_angle_rad,
            #     - self.SV.PT.max_steering_angle + np.radians(self.SV.PT.current_yaw)%(2*np.pi),
            #     self.SV.PT.max_steering_angle + np.radians(self.SV.PT.current_yaw)%(2*np.pi)
            # )
            # self.SV.PT.steering_angle_rad += np.radians(self.SV.PT.current_yaw)



        # print(np.degrees(self.SV.PT.steering_target_angle_rad))

    def normalizeAngleRadian(self, angle):
        '''normalize angle to [-180, 180]'''
        # angle = angle%(2*np.pi)
        # angle = angle if angle - np.pi < 0 else angle - (2*np.pi)
        # return angle

        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


    def normalizeAngleDegree(self, angle):
        '''normalize angle to [-180, 180]'''
        # angle = angle%360
        # angle = angle if angle - 180 < 0 else angle - 360
        # return angle

        while angle > 180:
            angle -= 2.0 * 180

        while angle < -180:
            angle += 2.0 * 180

        return angle

    def findDirection(self):
        # global_angle_difference = np.degrees(np.arctan2(
        #     self.difference_y[self.SV.PT.target_index], self.difference_x[self.SV.PT.target_index]
        # ))%360
        global_angle_difference = np.degrees(np.arctan2(
            -self.difference_y[self.SV.PT.target_index], -self.difference_x[self.SV.PT.target_index]
        ))%360

        local_angle_difference = (180 - self.SV.PT.current_yaw + global_angle_difference)%360
        direction = 1 if local_angle_difference - 180 < 0 else -1
        # print(global_angle_difference ,local_angle_difference, direction)


        return direction

class StanleyController_sim(StanleyController):
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        super().__init__(self.SV)

    def next_target(self):
        self.last_target_index = self.SV.PT.target_index
        # Calculate car head position
        head_x = self.SV.PT.current_x + self.SV.PT.distance_front_rear_wheel*np.cos(np.radians(self.SV.PT.current_yaw))
        head_y = self.SV.PT.current_y + self.SV.PT.distance_front_rear_wheel*np.sin(np.radians(self.SV.PT.current_yaw))
        self.difference_x = self.SV.CF.fitted_route_x - head_x
        self.difference_y = self.SV.CF.fitted_route_y - head_y
        self.difference = np.hypot(self.difference_x, self.difference_y)
        self.SV.PT.target_distance = np.min(self.difference)
        self.SV.PT.target_index = np.where(self.difference == self.SV.PT.target_distance)
        # if len(self.SV.PT.target_index) > 1:
            # self.SV.PT.target_index = self.SV.PT.target_index[0]


        # Target index in the format of (([], ""), ([], ""), ...)
        self.SV.PT.target_index = self.SV.PT.target_index[0][0]
        self.SV.PT.target_position = [self.SV.CF.fitted_route_x[self.SV.PT.target_index], self.SV.CF.fitted_route_y[self.SV.PT.target_index]]



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
