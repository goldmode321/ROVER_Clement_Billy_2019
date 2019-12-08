import numpy
import math



class MapBuilder():
    def __init__(self, rover_shared_variable):
        self.MAP = rover_shared_variable.MAP
        self.LOBS = rover_shared_variable.LOBS
        self.GOBS = rover_shared_variable.GOBS
        self.LI = rover_shared_variable.LI
        self.VI = rover_shared_variable.VI
        self.CAL = rover_shared_variable.CAL

    def calculate_local_obstacle(self):
        self.LOBS.local_obstacle_x = numpy.round(numpy.cos(self.LI.lidar_angle - \
            self.VI.vision_angle_radian + 0.5*math.pi)*\
            self.LI.lidar_radius + self.VI.vision_x, 0)
        self.LOBS.local_obstacle_y = numpy.round(numpy.sin(self.LI.lidar_angle - \
            self.VI.vision_angle_radian + 0.5*math.pi)*\
            self.LI.lidar_radius + self.VI.vision_y, 0)

    def calculate_arrow(self):
        self.MAP.arrow_x = [self.VI.vision_x, self.VI.vision_x + 200*math.cos(-self.VI.vision_angle_radian+0.5*math.pi)]
        self.MAP.arrow_y = [self.VI.vision_y, self.VI.vision_y + 200*math.sin(-self.VI.vision_angle_radian+0.5*math.pi)]

    def calculate_vision_xy_angle(self):
        self.VI.vision_x = self.VI.vision_data[0] * self.CAL.calibrate_x_multi + self.CAL.calibrate_x
        self.VI.vision_y = self.VI.vision_data[1] * self.CAL.calibrate_y_multi + self.CAL.calibrate_y
        self.VI.vision_angle_radian = math.radians(
            self.VI.vision_data[2] * self.CAL.calibrate_angle_multi + self.CAL.calibrate_angle
            )

    def get_global_obstacle(self):
        # combine it into form of (x1,y1), (x2, y2),...
        temp_global_obstacle = numpy.vstack((self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)).T
        # Filt old obstacle data
        new_global_obstacle = numpy.asarray([i for i in temp_global_obstacle if not i in self.GOBS.global_obstacle])
        self.GOBS.global_obstacle = numpy.append(self.GOBS.global_obstacle, new_global_obstacle)
        self.GOBS.global_obstacle_x = self.GOBS.global_obstacle[:, 0]
        self.GOBS.global_obstacle_y = self.GOBS.global_obstacle[:, 1]
        self.GOBS.global_obstacle_buffer = [self.GOBS.global_obstacle_buffer, new_global_obstacle]
        print(len(self.GOBS.global_obstacle_buffer))

    def get_x_axis_limit(self):
        ''' return value set for set_xlim()'''
        return numpy.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).min(), \
            numpy.concatenate((self.GOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).max()

    def get_y_axis_limit(self):
        ''' return value set for set_ylim() '''
        return numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).min(), \
                numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).max()
