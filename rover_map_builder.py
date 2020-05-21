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
        self.COM = self.SV.COM

        self.COM.command_map_builder['mbg'] = self.get_global_obstacle

    def calculate_local_obstacle(self):
        self.LOBS.local_obstacle_x = np.round(np.cos(self.LI.lidar_angle - \
            self.VI.vision_angle_radian + 0.5*np.pi)* \
            self.LI.lidar_radius + self.VI.vision_x, 0)
        self.LOBS.local_obstacle_y = np.round(np.sin(self.LI.lidar_angle - \
            self.VI.vision_angle_radian + 0.5*math.pi)*\
            self.LI.lidar_radius + self.VI.vision_y, 0)

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
        print(len(self.GOBS.global_obstacle_x))


#     def get_x_axis_limit(self):
#         ''' return value set for set_xlim()'''
#         return np.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).min(), \
#             np.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).max()

#     def get_y_axis_limit(self):
#         ''' return value set for set_ylim() '''
#         return np.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).min(), \
#                 np.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).max()


# class LocalObsBuilderThread(threading.Thread):
#     def __init__(self, MapBuilder):
#         super().__init__(daemon=True)
#         self.map_builder = MapBuilder
#         self.run_flag = False
#         self.start()

#     def stop(self):
#         self.run_flag = False

#     def run(self):
#         self.run_flag = True
#         # i = 0
#         while self.run_flag:
#             self.map_builder.calculate_local_obstacle()
#             time.sleep(self.map_builder.MAP.local_obs_update_delay)


