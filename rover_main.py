import time
import traceback
import subprocess
import threading
import multiprocessing
import logging
import numpy as np
import Adafruit_PCA9685

import rover_socket
import rover_shared_variable
import rover_rplidar
import rover_vision
import rover_car_control
import rover_curve_fitting
import rover_pathplanning
import rover_pathtracking
import rover_map_builder
import rover_communication


class Main():
    '''Communication center Main(auto_start=True)'''
    def __init__(self, auto_start=True):
        '''If auto_start is False, program automatically run __init__ only'''
        logging.basicConfig(filename='Main.log', filemode='w', level=logging.INFO)

        self.SV = rover_shared_variable.SharedVariables()
        self.VI = self.SV.VI
        self.LI = self.SV.LI
        self.CC = self.SV.CC
        self.MAP = self.SV.MAP
        self.CAL = self.SV.CAL
        self.AS = self.SV.AS
        self.PT = self.SV.PT
        self.ROV = self.SV.ROV
        self.LOBS = self.SV.LOBS
        self.GOBS = self.SV.GOBS
        self.CF = self.SV.CF
        self.COM = self.SV.COM

        # Main initial variables
        self.auto_start = auto_start
        self.main_receive = []

        # Commander initail variables
        self.commander_server_run = False
        # Vision initial variables
        self.vision_build_map_mode = False
        self.vision_use_map_mode = False
        # Algorithm initial variables
        self.algorithm_run = False
        self.algorithm_server_run = False
        # GUI initial variables
        self.gui_server_run = False
        self.gui_receive = []


        self.COM.command_rover = {
            'exit all':self._exit_all, 'svgobs':self._svgobs,
            "import":self._import,
        }

        self.COM.command_lidar = {
            'li':self._li,
        }

        self.COM.command_vision = {
            'exit v':self.end_vision, 'vi':self.vision_init,
        }

        self.COM.command_car_control = {
            'cci':self.car_control_init
        }

        self.COM.command_path_tracking = {'pti':self.path_tracking_init}
        self.COM.command_path_planning = {
            'pp':self._pp, 'pps':self._pp_s, 'ppe':self._pp_e, 'ppv':self._pp_v,
            "ppr":self._pp_r,
        }

        self.COM.command_calibration = {
            "cal":self._cal,
        }

        self.path_planning = rover_pathplanning.AstarPathPlanning(self.SV)
        self.curve_fitting = rover_curve_fitting.CubicSpline(self.SV)
        self.map_builder = rover_map_builder.MapBuilder(self.SV)
        self.communication = rover_communication.Communication(self.SV)

        if self.auto_start:


            self.vision_init()
            self.lidar_init()
            self.car_control_init()
            self.path_tracking_init()

        self.main_main()




################## Vision ##############

    def vision_init(self):
        '''Initialize vision system and communication '''
        if not self.VI.vision_run:
            try:
                logging.info("Initialize vision")
                self.vision = rover_vision.Vision(self.SV)
                logging.info("Vision initiated\n")
            except:
                print('\nError from Main : vision_init \n')
                traceback.print_exc()
                logging.info('Main initializing fail at vision_init()')
                logging.exception("Got error : \n")
        else:
            print('Vision run already')

    def end_vision(self):
        '''End vision module system'''
        if self.VI.vision_run:
            self.vision.end()
            logging.info('Vision end')
        else:
            print('Vision already off')

################## LiDAR ######################

    def lidar_init(self):
        '''Initialize Lidar system and communication'''
        try:
            logging.info("Initialize lidar server\n")
            self.lidar = rover_rplidar.Lidar(self.SV)
            logging.info('Lidar initiated')

        except:
            print('\nError from Main : lidar_init\n')
            traceback.print_exc()
            logging.info('Main initializing fail at lidar_init()\n')
            logging.exception("Main initializing fail at lidar_init() : \n")

    def _li(self):
        if not self.LI.lidar_run:
            self.lidar_init()
        else:
            print('LiDAR run already')

    def end_lidar(self):
        '''End lidar system'''
        if self.LI.lidar_run:
            self.lidar.end()
            logging.info('LiDAR end')
        else:
            print('LiDAR already off')

########## CarControl #############
    def car_control_init(self):
        if not self.ROV.car_control_ready:
            self.car_control = rover_car_control.CarControl(self.SV)
        else:
            print('CarControl had started')


######## Path Tracking ############
    def path_tracking_init(self):
        if not self.ROV.path_tracking_ready:
            self.path_tracking = rover_pathtracking.PathTracking(self.SV)
        else:
            print("Path Tracking already standby")

############ Main main ###################
    def main_main(self):
        '''Main waiting for command'''
        self.ROV.rover_run = True
        while self.ROV.rover_run:
            try:
                command = input("\nPlease enter command , enter 'h' for _help : ")
                logging.info('Command : %s', command)
                if command in self.COM.command_rover:
                    self.COM.command_rover[command]() # Referenced from CommanderDictionary
                elif command in self.COM.command_vision:
                    self.COM.command_vision[command]()
                elif command in self.COM.command_lidar:
                    self.COM.command_lidar[command]()
                elif command in self.COM.command_car_control:
                    self.COM.command_car_control[command]()
                elif command in self.COM.command_path_planning:
                    self.COM.command_path_planning[command]()
                elif command in self.COM.command_path_tracking:
                    self.COM.command_path_tracking[command]()
                elif command in self.COM.command_map_builder:
                    self.COM.command_map_builder[command]()
                elif command in self.COM.command_calibration:
                    self.COM.command_calibration[command]()
                else:
                    print('Unknown Command')
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.end_main_all()
                self.ROV.rover_run = False
            except:
                print('Critical error happened on Main , all programs have to be shutdown')
                self.end_main_all()
                print('\nError from Main : main_main \n')
                traceback.print_exc()
                logging.exception('Got error :')
                self.ROV.rover_run = False


    def end_main_all(self):
        '''Close all system except for tnc_main'''
        self.end_lidar()
        self.end_vision()
        self.communication.end_gui_server()
        self.path_tracking.stop()
        self.car_control.stop()



############ Command relative function ###########
    def _exit_all(self):
        self.end_main_all()
        self.ROV.rover_run = False

    def _svgobs(self):
        name = input("Save name")
        if name != "":
            np.savez(
                name, 
                gobs=self.GOBS.global_obstacle,
                gobsx=self.GOBS.global_obstacle_x,
                gobsy=self.GOBS.global_obstacle_y,
                vid=self.VI.vision_map_id,
                calx=self.CAL.calibrate_x,
                calxm=self.CAL.calibrate_x_multi,
                caly=self.CAL.calibrate_y,
                calym=self.CAL.calibrate_y_multi,
                cala=self.CAL.calibrate_angle,
                calam=self.CAL.calibrate_angle_multi,
            )
            print("global map save to {}.npz".format(name))
        print("Abort save process")

    def _import(self):
        try:
            name = input("Import name") + ".npz"
            if name != "":
                data = np.load("./map/{}".format(name))
                self.GOBS.global_obstacle = data["gobs"]
                self.GOBS.global_obstacle_x = data["gobsx"]
                self.GOBS.global_obstacle_y = data["gobsy"]
                self.VI.vision_map_id = int(data["vid"])
                self.CAL.calibrate_x = int(data["calx"])
                self.CAL.calibrate_x_multi = float(data["calxm"])
                self.CAL.calibrate_y = int(data["caly"])
                self.CAL.calibrate_y_multi = float(data["calym"])
                self.CAL.calibrate_angle = int(data["cala"])
                self.CAL.calibrate_angle_multi = float(data["calam"])
                print("Import map {} complete".format(name))
        except:
            traceback.print_exc()


######## Calibration ##############
    def _cal(self):
        self.CAL.calibrate_x = int(input("Calibrate x (mm): "))
        self.CAL.calibrate_x_multi = abs(float(input("Calibrate x multi (%): ")))/100
        self.CAL.calibrate_y = int(input("Calibrate y (mm): "))
        self.CAL.calibrate_y_multi = abs(float(input("Calibrate y multi (%): ")))/100
        self.CAL.calibrate_angle = int(input("Calibrate angle (mm): "))
        self.CAL.calibrate_angle_multi = abs(float(input("Calibrate angle multi (%): ")))/100
        self.CAL.calibrate_dis_lv = int(input("horizontal distance between lidar and vision"))



############# Path Planning ##############
    def _pp(self):
        print("Start Planning")
        self._pp_v()
        self.path_planning.planning()
        self.curve_fitting.fitting()

    def _pp_s(self):
        self.AS.start_x = int(input("Start X Pos : "))
        self.AS.start_y = int(input("Start Y Pos : "))
        self.AS.attitude[0] = int(input("Start Angle (deg) : "))

    def _pp_e(self):
        self.AS.end_x = int(input("End X Pos : "))
        self.AS.end_y = int(input("End Y Pos : "))

    def _pp_v(self):
        self.AS.start_x = int(self.VI.vision_x)
        self.AS.start_y = int(self.VI.vision_y)
        self.AS.attitude[0] = int(self.VI.vision_angle + 90)

    def _pp_r(self):
        self.AS.step_unit = int(input("Step unit (cm) :"))
        self.AS.rover_size = int(input("Rover size (cm) :"))





if __name__ == "__main__":
    Main()
