import numpy
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

    def calculate_local_obstacle(self):

        if len(self.LI.lidar_angle) == len(self.LI.lidar_radius):
            self.LOBS.local_obstacle_x = numpy.round(numpy.cos(self.LI.lidar_angle - \
                self.VI.vision_angle_radian + 0.5*math.pi)* \
                self.LI.lidar_radius + self.VI.vision_x, 0)
            self.LOBS.local_obstacle_y = numpy.round(numpy.sin(self.LI.lidar_angle - \
                self.VI.vision_angle_radian + 0.5*math.pi)*\
                self.LI.lidar_radius + self.VI.vision_y, 0)
        else:
            # print("Race condition happen in map builder")
            pass

    def calculate_arrow(self):
        self.MAP.arrow_x = [self.VI.vision_x, self.VI.vision_x + 200*math.cos(-self.VI.vision_angle_radian+0.5*math.pi)]
        self.MAP.arrow_y = [self.VI.vision_y, self.VI.vision_y + 200*math.sin(-self.VI.vision_angle_radian+0.5*math.pi)]


    def get_global_obstacle(self):
        # # combine it into form of (x1,y1), (x2, y2),...
        # temp_global_obstacle = numpy.vstack((self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)).T
        # # Filt old obstacle data
        # new_global_obstacle = numpy.asarray([i for i in temp_global_obstacle if not i in self.GOBS.global_obstacle])
        # if len(new_global_obstacle) > 0 and len(self.GOBS.global_obstacle) > 0:
        #     self.GOBS.global_obstacle = numpy.concatenate((self.GOBS.global_obstacle, new_global_obstacle))
        # elif len(self.GOBS.global_obstacle) == 0:
        #     self.GOBS.global_obstacle = new_global_obstacle
        # self.GOBS.global_obstacle_x = self.GOBS.global_obstacle[:, 0]
        # self.GOBS.global_obstacle_y = self.GOBS.global_obstacle[:, 1]
        # # self.GOBS.global_obstacle_buffer = [self.GOBS.global_obstacle_buffer, new_global_obstacle]
        # print(len(self.GOBS.global_obstacle_x))

        # combine it into form of (x1,y1), (x2, y2),...
        temp_global_obstacle = numpy.vstack((self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)).T
        # Find new Obstacle index
        temp_bool_index = numpy.isin(temp_global_obstacle, self.GOBS.global_obstacle, invert=True)
        temp_index = numpy.where(temp_bool_index)
        temp_index = numpy.unique(temp_index[0])
        # Filt old obstacle data
        new_global_obstacle = temp_global_obstacle[temp_index]
        if len(new_global_obstacle) > 0 and len(self.GOBS.global_obstacle) > 0:
            self.GOBS.global_obstacle = numpy.concatenate((self.GOBS.global_obstacle, new_global_obstacle))
        elif len(self.GOBS.global_obstacle) == 0:
            self.GOBS.global_obstacle = new_global_obstacle
        self.GOBS.global_obstacle_x = self.GOBS.global_obstacle[:, 0]
        self.GOBS.global_obstacle_y = self.GOBS.global_obstacle[:, 1]
        # self.GOBS.global_obstacle_buffer = [self.GOBS.global_obstacle_buffer, new_global_obstacle]
        print(len(self.GOBS.global_obstacle_x))


    def get_x_axis_limit(self):
        ''' return value set for set_xlim()'''
        return numpy.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).min(), \
            numpy.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).max()

    def get_y_axis_limit(self):
        ''' return value set for set_ylim() '''
        return numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).min(), \
                numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).max()


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
            self.map_builder.calculate_local_obstacle()
            time.sleep(self.MAP.local_obs_update_delay)
            # i += 1
            # if i > 100:
            #     print("LOBS running")
            #     i = 0

