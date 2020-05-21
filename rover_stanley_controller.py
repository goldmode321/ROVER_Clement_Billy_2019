import numpy as np


class StanleyController:
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        self.PT = self.SV.PT
        self.VI = self.SV.VI
        self.last_target_index = 0
        self.difference = None
        self.difference_x = None
        self.difference_y = None


    def update_state(self):
        self.PT.current_x += self.PT.velocity * np.cos(np.radians(self.PT.current_yaw)) * self.PT.interval/1000
        self.PT.current_y += self.PT.velocity * np.sin(np.radians(self.PT.current_yaw)) * self.PT.interval/1000
        self.PT.current_yaw += np.degrees(self.PT.velocity / self.PT.distance_front_rear_wheel *\
            np.tan(self.PT.real_steer_rad) * self.PT.interval/1000)
        # self.PT.current_yaw = self.PT.current_yaw % 2*np.pi # normalize yaw to 0~2 pi
        self.PT.velocity += self.PT.acceleration * self.PT.interval/1000

    def update_state_real(self):
        self.PT.current_x = self.VI.vision_x
        self.PT.current_y = self.VI.vision_y
        self.PT.current_yaw = - self.VI.vision_angle + 90


    def next_target(self, manual_mode=False):
        self.last_target_index = self.PT.target_index
        # Calculate car head position
        head_x = self.PT.current_x + self.PT.distance_front_rear_wheel*np.cos(np.radians(self.PT.current_yaw))
        head_y = self.PT.current_y + self.PT.distance_front_rear_wheel*np.sin(np.radians(self.PT.current_yaw))
        self.difference_x = self.SV.CF.fitted_route_x - head_x
        self.difference_y = self.SV.CF.fitted_route_y - head_y
        self.difference = np.hypot(self.difference_x, self.difference_y)
        self.PT.target_distance = np.min(self.difference)
        self.PT.target_index = np.where(self.difference == self.PT.target_distance)
        # if len(self.PT.target_index) > 1:
            # self.PT.target_index = self.PT.target_index[0]


        # Target index in the format of (([], ""), ([], ""), ...)
        self.PT.target_index = self.PT.target_index[0][0]
        if self.last_target_index >= self.PT.target_index and not manual_mode:
            self.PT.target_index = self.last_target_index
        self.PT.target_position = [self.SV.CF.fitted_route_x[self.PT.target_index], self.SV.CF.fitted_route_y[self.PT.target_index]]



        front_axle_vec = [np.cos(np.radians(self.PT.current_yaw) + np.pi / 2),
                        np.sin(np.radians(self.PT.current_yaw) + np.pi / 2)]
        self.error_front_axle = np.dot(
            [self.difference_x[self.PT.target_index], self.difference_y[self.PT.target_index]], front_axle_vec)



    def calculate_command(self):
        self.next_target()
        if self.PT.velocity >= 0:
            self.PT.theta_e = self.PT.theta_e_gain * self.normalize_angle_rad(self.normalize_angle_deg(np.radians(self.PT.current_yaw)) - self.SV.CF.fitted_route_yaw_rad[self.PT.target_index])
            self.PT.theta_e_deg = np.rad2deg(self.PT.theta_e)

            self.PT.theta_d = np.arctan2(self.PT.theta_d_gain*self.error_front_axle, self.PT.velocity)
            self.PT.theta_d_deg = np.rad2deg(self.PT.theta_d)
            self.PT.steering_target_angle_rad = self.PT.theta_e + self.PT.theta_d


            self.PT.steering_target_angle_deg = np.degrees(self.PT.steering_target_angle_rad)

            self.PT.real_steer_deg = np.clip(
                self.PT.steering_target_angle_deg,
                - self.PT.max_steering_angle,
                self.PT.max_steering_angle
            )
            self.PT.real_steer_rad = np.radians(self.PT.real_steer_deg)

            self.PT.real_steer_command = int(405 + self.PT.real_steer_deg/3*8)

            self.PT.steering_angle_deg = self.PT.real_steer_deg + self.PT.current_yaw
            self.PT.steering_angle_rad = np.deg2rad(self.PT.steering_angle_deg)

        elif self.PT.velocity < 0:
            self.PT.theta_e = self.normalize_angle_rad(np.radians(self.PT.current_yaw) - self.SV.CF.fitted_route_yaw_rad[self.PT.target_index])
            self.PT.theta_e_deg = np.degrees(self.PT.theta_e)

            self.PT.theta_d = np.arctan2(self.PT.theta_d_gain*self.error_front_axle, self.PT.velocity)
            self.PT.theta_d_deg = np.rad2deg(self.PT.theta_d)
            self.PT.steering_target_angle_rad = self.PT.theta_e - self.PT.theta_d

            self.PT.steering_target_angle_rad = self.normalize_angle_rad(self.PT.steering_target_angle_rad)

            self.PT.steering_target_angle_deg = np.rad2deg(self.PT.steering_target_angle_rad)

            self.PT.real_steer_deg = - np.clip(
                 - self.PT.steering_target_angle_deg,
                - self.PT.max_steering_angle,
                self.PT.max_steering_angle
            )

            self.PT.real_steer_rad = np.radians(self.PT.real_steer_deg)

            self.PT.real_steer_command = int(405 + self.PT.real_steer_deg/3*8)

            self.PT.steering_angle_deg = self.PT.real_steer_deg + self.PT.current_yaw
            self.PT.steering_angle_rad = np.radians(self.PT.steering_angle_deg)

        # print("e : {} d : {} steer_target : {} steer : {} real : {}".format(
        #     self.PT.theta_e_deg, self.PT.theta_d_deg, self.PT.steering_target_angle_deg,
        #     self.PT.steering_angle_deg, self.PT.real_steer_deg
        # ))


    def normalize_angle_rad(self, angle):
        '''normalize angle to [-180, 180]'''
        # angle = angle%(2*np.pi)
        # angle = angle if angle - np.pi < 0 else angle - (2*np.pi)
        # return angle

        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


    def normalize_angle_deg(self, angle):
        '''normalize angle to [-180, 180]'''
        # angle = angle%360
        # angle = angle if angle - 180 < 0 else angle - 360
        # return angle

        while angle > 180:
            angle -= 2.0 * 180

        while angle < -180:
            angle += 2.0 * 180

        return angle

    def find_direction(self):
        # global_angle_difference = np.degrees(np.arctan2(
        #     self.difference_y[self.PT.target_index], self.difference_x[self.PT.target_index]
        # ))%360
        global_angle_difference = np.degrees(np.arctan2(
            -self.difference_y[self.PT.target_index], -self.difference_x[self.PT.target_index]
        ))%360

        local_angle_difference = (180 - self.PT.current_yaw + global_angle_difference)%360
        direction = 1 if local_angle_difference - 180 < 0 else -1
        # print(global_angle_difference ,local_angle_difference, direction)


        return direction

class StanleyController_sim(StanleyController):
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        super().__init__(self.SV)




    def next_target(self):
        self.last_target_index = self.PT.target_index
        # Calculate car head position
        head_x = self.PT.current_x + self.PT.distance_front_rear_wheel*np.cos(np.radians(self.PT.current_yaw))
        head_y = self.PT.current_y + self.PT.distance_front_rear_wheel*np.sin(np.radians(self.PT.current_yaw))
        self.difference_x = self.SV.CF.fitted_route_x - head_x
        self.difference_y = self.SV.CF.fitted_route_y - head_y
        self.difference = np.hypot(self.difference_x, self.difference_y)
        self.PT.target_distance = np.min(self.difference)
        self.PT.target_index = np.where(self.difference == self.PT.target_distance)
        # if len(self.PT.target_index) > 1:
            # self.PT.target_index = self.PT.target_index[0]


        # Target index in the format of (([], ""), ([], ""), ...)
        self.PT.target_index = self.PT.target_index[0][0]
        self.PT.target_position = [self.SV.CF.fitted_route_x[self.PT.target_index], self.SV.CF.fitted_route_y[self.PT.target_index]]

        front_axle_vec = [np.cos(np.radians(self.PT.current_yaw) + np.pi / 2),
                        np.sin(np.radians(self.PT.current_yaw) + np.pi / 2)]
        self.error_front_axle = np.dot(
            [self.difference_x[self.PT.target_index], self.difference_y[self.PT.target_index]], front_axle_vec)