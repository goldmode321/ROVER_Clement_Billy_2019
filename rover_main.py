import time
import traceback
import subprocess
import threading
import multiprocessing
import logging
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



        self.path_planning = rover_pathplanning.AstarPathPlanning(self.SV)
        self.curve_fitting = rover_curve_fitting.CubicSpline(self.SV)
        self.map_builder = rover_map_builder.MapBuilder(self.SV)
        self.communication = rover_communication.Communication(self.SV)


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


        self.COM.command_rover = {'exit all':self._exit_all, 'next':self._next}

        self.COM.command_lidar = {
            'exit l':self._exit_l, 'li':self._li, 'gld':self._gld, 'next':self._next, \
                'lr':self._lr, 'ls':self._ls,
        }

        self.COM.command_vision = {
            'exit v':self._exit_v, 'vi':self._vi, 'vs':self._vs, 'gs':self._gs, 'al':self._al, \
                'cc':self._vcc, 'sv':self._sv, 'vrs':self._vrs, 'gp c':self._gp_c, \
                    'gp exit':self._gp_exit, 'gp':self._gp, 'bm':self._bm, 'um':self._um, \
                        'next':self._next
        }

        self.COM.command_car_control = {
            'ccs':self._cc_s, 'cci':self._cc_i, 'cct':self._cc_t, 'ccstop':self._cc_stop,
        }

        self.COM.command_path_tracking = {
            'pti':self._pt_i, 'ptm':self._pt_m, 'pta':self._pt_a, 'ptstop':self._pt_stop,
            'ptswi':self._pt_swi,
        }
        self.COM.command_path_planning = {
            'pp':self._pp, 'pps':self._pp_s, 'ppe':self._pp_e, 'ppv':self._pp_v,
        }

        self.COM.command_map_builder = {
            'mb start':self._mb_start, 'mb stop':self._mb_stop, 'mbg':self._mb_g
        }

        self.COM.command_calibration = {
            "cal":self._cal,
        }


        if self.auto_start:


            self.vision_init()
            self.lidar_init()
            self.car_control_init()
            self.path_tracking_init()
            self.localObsBuilderInit()

        self.main_main()




################## Vision ##############

    def vision_init(self):
        '''Initialize vision system and communication '''
        try:
            logging.info("Initialize vision")
            self.vision = rover_vision.Vision(self.SV)
            logging.info("Vision initiated\n")
        except:
            print('\nError from Main : vision_init \n')
            traceback.print_exc()
            logging.info('Main initializing fail at vision_init()')
            logging.exception("Got error : \n")


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

######## Map Builder ##########
    def localObsBuilderInit(self):
        self.local_obs_builder = rover_map_builder.LocalObsBuilderThread(self.SV)

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
        self._pt_stop()
        self._cc_stop()





    def main_show_vision_data(self):
        '''Show vision data'''
        self.main_show_vision_data_run = True
        while self.main_show_vision_data_run:
            try:
                print('status : {} | x : {} | y : {} | theta : {} | Use Ctrl+C or enter any key to end current process : '\
                    .format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_angle))
                time.sleep(0.1)
            except:
                print('\nError from Main : main_send_vision_data_to_commander\n')
                traceback.print_exc()
                logging.exception('Got error : ')
                self.main_show_vision_data_run = False


    def show_vision_status(self):
        '''Show message depends on vision status'''
        if self.VI.vision_status == 0:
            print("Vision module status : {} | Vision module is booting".format(self.VI.vision_status))
        elif self.VI.vision_status == 1:
            print("Vision module status : {} | Vision module is waiting for 'st $mapid' command".format(self.VI.vision_status))
        elif self.VI.vision_status == 2:
            print("Vision module status : {} | Vision module is loading data ".format(self.VI.vision_status))
        elif self.VI.vision_status == 3:
            print('Vision module status : {} | Please move slowly, fp-slam is searching a set of best images to initialize'.format(self.VI.vision_status))
        elif self.VI.vision_status == 4:
            print('Vision module status : {} | System is working normaaly'.format(self.VI.vision_status))
        elif self.VI.vision_status == 5:
            print('Vision module status : {} | Lost Lost Lost'.format(self.VI.vision_status))
        else:
            print('Unknown status code : {}'.format(self.VI.vision_status))






############ Command relative function ###########
    def _exit_all(self):
        self.end_main_all()
        self.ROV.rover_run = False
    def _exit_l(self):
        self.end_lidar()
    def _exit_v(self):
        self.end_vision()

######## Calibration ##############
    def _cal(self):
        self.CAL.calibrate_x = int(input("Calibrate x (mm): "))
        self.CAL.calibrate_x_multi = abs(float(input("Calibrate x multi (%): ")))/100
        self.CAL.calibrate_y = int(input("Calibrate y (mm): "))
        self.CAL.calibrate_y_multi = abs(float(input("Calibrate y multi (%): ")))/100
        self.CAL.calibrate_angle = int(input("Calibrate angle (mm): "))
        self.CAL.calibrate_angle_multi = abs(float(input("Calibrate angle multi (%): ")))/100
        self.CAL.calibrate_difference_between_lidar_and_vision = int(input("horizontal distance between lidar and vision"))

######### Map builder ###############
    def _mb_start(self):
        if self.local_obs_builder.isAlive():
            self.local_obs_builder = rover_map_builder.LocalObsBuilderThread(self.SV)

    def _mb_stop(self):
        self.local_obs_builder.stop()

    def _mb_g(self):
        self.map_builder.get_global_obstacle()

################ LiDAR ###############
    def _li(self):
        if not self.LI.lidar_run:
            self.lidar_init()
        else:
            print('LiDAR run already')

    def _lr(self):
        self.lidar.lidar_run()

    def _ls(self):
        self.lidar.lidar_stop()

    def _gld(self):
        print(self.LI.lidar_data)

    ################ Vision #############
    def _vi(self):
        if not self.VI.vision_run:
            self.vision_init()
        else:
            print('Vision run already')
    def _vs(self):
        print('Vision server : {}\n'.format(self.VI.vision_run))
    def _gs(self):
        self.show_vision_status()
    def _sv(self):
        self.vision.save()
    def _al(self):
        self.vision.alive()
    def _vcc(self):
        self.vision.check_cpu_speed()
    def _vrs(self):
        self.vision.reset()
    def _gp_c(self):
        self.main_show_vision_data()
    def _gp_exit(self):
        self.main_show_vision_data_run = False
    def _gp(self):
        print('status : {} | x : {} | y : {} | theta : {} '.format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_angle))
    def _bm(self):
        try:
            mapid = int(input('MapID : '))
            self.vision.build_map(mapid)
            try:
                input("Use Ctrl+C or enter any key to end current process : ")
                self.vision.save()
                self.vision.reset()
            except KeyboardInterrupt:
                self.vision.save()
                self.vision.reset()
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')
    def _um(self):
        try:
            mapid = int(input('MapID : '))
            self.vision.use_map(mapid)
            try:
                input("Use Ctrl+C or enter any key to end current process : ")
                self.vision.save()
                self.vision.reset()
            except KeyboardInterrupt:
                self.vision.save()
                self.vision.reset()
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')
    def _next(self):
        pass

######### Car Control #########
    def _cc_i(self):
        self.car_control_init()
    
    def _cc_t(self):
        self.CC.car_control_steer = int(input("Input range 320-405-490 : "))

    def _cc_s(self):
        self.car_control.start()

    def _cc_stop(self):
        self.car_control.stop()

############# Path Planning ##############
    def _pp(self):
        print("Start Planning")
        self.path_planning.planning()
        self.curve_fitting.fitting()

    def _pp_s(self):
        self.AS.start_x = int(input("Start X Pos : "))
        self.AS.start_y = int(input("Start Y Pos : "))
        self.AS.attitude[0] = int(input("Start Angle (deg) : "))

    def _pp_e(self):
        self.AS.end_x = int(input("End X Pos : "))
        self.AS.end_x = int(input("End X Pos : "))

    def _pp_v(self):
        self.AS.start_x = int(self.VI.vision_x)
        self.AS.start_y = int(self.VI.vision_y)
        self.AS.attitude[0] = int(self.VI.vision_angle)

################## Path Tracking #############
    def _pt_i(self):
        self.path_tracking_init()

    def _pt_m(self):
        '''Path tracking by manual forward and back'''
        self.path_tracking.manualControl()

    def _pt_a(self):
        '''Path tracking by auto forward'''
        self.path_tracking.autoControl()

    def _pt_stop(self):
        self.path_tracking.stop()

    def _pt_swi(self):
        print("stanley, stanley_sim : ")
        method = input("Choose : ")
        self.path_tracking.switchMethod(method)


if __name__ == "__main__":
    Main()
