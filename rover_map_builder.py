import numpy as np
import math
import threading
import time
import logging



class MapBuilder:
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        self.MAP = self.SV.MAP
        self.LOBS = self.SV.LOBS
        self.GOBS = self.SV.GOBS
        self.LI = self.SV.LI
        self.VI = self.SV.VI
        self.CAL = self.SV.CAL

    def calculateGlobalVision(self):
        self.VI.vision_angle = 90 - (self.VI.temp_vision_angle_deg * self.CAL.calibrate_angle_multi) + self.CAL.calibrate_angle
        self.VI.vision_angle_radian = math.radians(self.VI.vision_angle)
        self.VI.vision_x = round((self.VI.temp_vision_x - self.CAL.calibrate_dis_lv * np.sin(self.VI.vision_angle_radian))* \
            self.CAL.calibrate_x_multi + self.CAL.calibrate_x, 1)
        self.VI.vision_y = round((self.VI.temp_vision_y - self.CAL.calibrate_dis_lv * np.cos(self.VI.vision_angle_radian))* \
            self.CAL.calibrate_y_multi + self.CAL.calibrate_y, 1)



    def calculate_local_obstacle(self):

        if len(self.LI.lidar_angle) == len(self.LI.lidar_radius):
            self.LOBS.local_obstacle_x = np.round(np.cos(self.LI.lidar_angle - \
                self.VI.vision_angle_radian) * self.LI.lidar_radius + self.VI.vision_x, 0
            )
            self.LOBS.local_obstacle_y = np.round(np.sin(self.LI.lidar_angle - \
                self.VI.vision_angle_radian) * self.LI.lidar_radius + self.VI.vision_y, 0
            )
        else:
            # print("Race condition happen in map builder")
            pass

    def calculate_arrow(self):
        self.MAP.arrow_x = [self.VI.vision_x, self.VI.vision_x + 200*math.cos(-self.VI.vision_angle_radian+0.5*math.pi)]
        self.MAP.arrow_y = [self.VI.vision_y, self.VI.vision_y + 200*math.sin(-self.VI.vision_angle_radian+0.5*math.pi)]


    def get_global_obstacle(self):

        # combine it into form of (x1,y1), (x2, y2),...
        temp_global_obstacle = np.vstack((self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)).T
        # Find new Obstacle index
        temp_bool_index = np.isin(temp_global_obstacle, self.GOBS.global_obstacle, invert=True)
        temp_index = np.where(temp_bool_index)
        temp_index = np.unique(temp_index[0])
        # Filt old obstacle data
        new_global_obstacle = temp_global_obstacle[temp_index]
        if len(new_global_obstacle) > 0 and len(self.GOBS.global_obstacle) > 0:
            self.GOBS.global_obstacle = np.concatenate((self.GOBS.global_obstacle, new_global_obstacle))
        elif len(self.GOBS.global_obstacle) == 0:
            self.GOBS.global_obstacle = new_global_obstacle
        self.GOBS.global_obstacle_x = self.GOBS.global_obstacle[:, 0]
        self.GOBS.global_obstacle_y = self.GOBS.global_obstacle[:, 1]
        # self.GOBS.global_obstacle_buffer = [self.GOBS.global_obstacle_buffer, new_global_obstacle]
        print("len(lobs) : {} len(newobs) : {} len(gobs) : {}".format(len(temp_global_obstacle), len(new_global_obstacle), len(self.GOBS.global_obstacle_x)))


    def get_x_axis_limit(self):
        ''' return value set for set_xlim()'''
        return np.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).min(), \
            np.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).max()

    def get_y_axis_limit(self):
        ''' return value set for set_ylim() '''
        return np.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).min(), \
                np.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).max()


class LocalObsBuilderThread(threading.Thread):
    def __init__(self, SharedVariables):
        self.SV = SharedVariables
        self.MAP = self.SV.MAP
        super().__init__(daemon=True)
        self.map_builder = MapBuilder(self.SV)
        self.run_flag = False
        self.start()

    def stop(self):
        self.run_flag = False

    def run(self):
        self.run_flag = True
        # i = 0
        while self.run_flag:
            try:
                self.map_builder.calculateGlobalVision()
                self.map_builder.calculate_local_obstacle()
            except Exception as e:
                print("Error in rover_map_builder.LocalObsBuilderThread.run()\n{}".format(e))
            time.sleep(self.MAP.local_obs_update_delay)


