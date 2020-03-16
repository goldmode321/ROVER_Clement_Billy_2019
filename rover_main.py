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



        self.path_planning = rover_pathplanning.AstarPathPlanning(self.SV)
        self.curve_fitting = rover_curve_fitting.CubicSpline(self.SV)


        # Main initial variables
        self.auto_start = auto_start
        self.main_run = False
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


        self.command_dictionary = {'exit all':self._exit_all, 'next':self._next}

        self.command_lidar_dictionary = {'exit l':self._exit_l, 'li':self._li, 'gld':self._gld, 'next':self._next}

        self.command_vision_dictionary = {'exit v':self._exit_v, 'vi':self._vi, 'vs':self._vs,\
                  'gs':self._gs, 'al':self._al, 'cc':self._vcc, 'sv':self._sv, 'vrs':self._vrs, 'gp c':self._gp_c, \
                        'gp exit':self._gp_exit, 'gp':self._gp, 'bm':self._bm, 'um':self._um, 'next':self._next}

        self.command_car_control_dictionary = {'ccs':self._cc_s, 'cci':self._cc_i, 'cct':self._cc_t}

        self.command_path_tracking_dictionary = {
            'pti':self._pt_i, 'ptm':self._pt_m, 'pta':self._pt_a, 'ptstop':self._pt_stop,
            'ptswi':self._pt_swi,
        }


        if self.auto_start:


            self.vision_init()
            self.lidar_init()
            self.car_control_init()
            self.gui_connection_init()
            self.path_tracking_init()

        self.main_main()

########### Move relative ############

########### Send to GUI ##############
    def gui_connection_init(self):
        self.gui_udp_client = rover_socket.UDP_server(50010, 0, "192.168.5.2")
        self.gui_send_status_udp_server = rover_socket.UDP_server(50012, 0, "192.168.5.2")
        self.gui_server_run = True
        self.gui_command = {"gss": self._gui_set_speed, "gbm": self._gui_bm, "gum":self._gui_um, \
            "gbms": self._gui_bm_stop, "gums":self._gui_um_stop}

        self.thread_gui = threading.Thread(target=self.gui_send_and_read, daemon=True)
        self.thread_gui.start()
        self.thread_gui_get_command = threading.Thread(target=self.gui_get_command, daemon=True)
        self.thread_gui_get_command.start()

    def gui_get_command(self):
        while self.gui_server_run:
            self.gui_get_command_udp_server = rover_socket.UDP_server(50013, 0, '192.168.5.2')
            time.sleep(0.1)
            self.gui_get_command_receive = self.gui_get_command_udp_server.recv_list()
            if self.gui_get_command_receive is not None:
                if self.gui_get_command_receive[0] in self.gui_command:
                    self.gui_command[self.gui_get_command_receive[0]]()
                elif self.gui_get_command_receive[0] in self.command_dictionary:
                    self.command_dictionary[self.gui_get_command_receive[0]]() # Referenced from CommanderDictionary
                elif self.gui_get_command_receive[0] in self.command_vision_dictionary:
                    self.command_vision_dictionary[self.gui_get_command_receive[0]]()
                elif self.gui_get_command_receive[0] in self.command_lidar_dictionary:
                    self.command_lidar_dictionary[self.gui_get_command_receive[0]]()
                elif self.command_car_control_dictionary[0] in self.command_car_control_dictionary:
                    self.command_car_control_dictionary[self.gui_get_command_receive[0]]()
                    print('Unknown Command')
            self.gui_get_command_udp_server.close()


    def gui_send_and_read(self):
        while self.gui_server_run:
            self.gui_receive = self.gui_udp_client.recv_list()
            self.gui_send_status_receive = self.gui_send_status_udp_server.recv_list()
            if self.gui_receive is not None:
                self.gui_udp_client.send_list_back([self.LI.lidar_data, self.VI.vision_data[0:4]])
            if self.gui_send_status_receive is not None:
                self.gui_send_status_udp_server.send_list_back([self.LI.lidar_USB_port, self.LI.lidar_state, self.LI.lidar_run, \
                    self.VI.vision_status, self.VI.vision_run, self.VI.vision_idle, self.main_run, self.CC.car_control_add_speed, \
                        self.vision_build_map_mode, self.vision_use_map_mode])
            time.sleep(0.05)



    def end_gui_server(self):
        self.gui_server_run = False
        self.thread_gui.join()
        self.thread_gui_get_command.join()
        self.gui_udp_client.close()
        self.gui_get_command_udp_server.close()
        self.gui_send_status_udp_server.close()

    def _gui_set_speed(self):
        self.CC.car_control_add_speed = self.gui_get_command_receive[1]
    def _gui_bm(self):
        # self.vision_server.send_list(['V', 'bm', self.gui_get_command_receive[1]])
        self.vision.build_map(self.gui_get_command_receive[1])
        # self.vision_idle = False
        self.vision_build_map_mode = True
    def _gui_bm_stop(self):
        print("Vision save, please wait")
        # self.vision_server.send_list(['V', 'sv'])
        self.vision.save()
        # self.main_receive = self.vision_server.recv_list()
        print("Vision reset, please wait")
        # self.vision_server.send_list(['V', 'rs'])
        self.vision.reset()
        # self.main_receive = self.vision_server.recv_list()
        print("Vision is now idling")
        # self.vision_idle = True
        self.vision_build_map_mode = False
    def _gui_um(self):
        # self.vision_server.send_list(['V', 'um', self.gui_get_command_receive[1]])
        self.vision.use_map(self.gui_get_command_receive[1])
        # self.vision_idle = False
        self.vision_use_map_mode = True
    def _gui_um_stop(self):
        print("Vision reset, please wait")
        # self.vision_server.send_list(['V', 'rs'])
        self.vision.reset()
        # self.main_receive = self.vision_server.recv_list()
        # self.vision_idle = True
        self.vision_use_map_mode = False


################## Vision ##############

    def vision_init(self):
        '''Initialize vision system and communication '''
        try:
            logging.info("Initialize vision server\n")
            self.vision = rover_vision.Vision(self.VI)
            logging.exception("Vision initiated")
        except:
            print('\nError from Main : vision_init \n')
            traceback.print_exc()
            logging.info('Main initializing fail at vision_init()\n')
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
            self.lidar = rover_rplidar.Lidar(self.LI)
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



############ Main main ###################
    def main_main(self):
        '''Main waiting for command'''
        self.main_run = True
        while self.main_run:
            try:
                command = input("\nPlease enter command , enter 'h' for _help : ")
                logging.info('Command : %s', command)
                if command in self.command_dictionary:
                    self.command_dictionary[command]() # Referenced from CommanderDictionary
                elif command in self.command_vision_dictionary:
                    self.command_vision_dictionary[command]()
                elif command in self.command_lidar_dictionary:
                    self.command_lidar_dictionary[command]()
                elif command in self.command_car_control_dictionary:
                    self.command_car_control_dictionary[command]()
                else:
                    print('Unknown Command')
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.end_main_all()
                self.main_run = False
            except:
                print('Critical error happened on Main , all programs have to be shutdown')
                self.end_main_all()
                print('\nError from Main : main_main \n')
                traceback.print_exc()
                logging.exception('Got error :')
                self.main_run = False

    def end_main_all(self):
        '''Close all system except for tnc_main'''

        self.end_lidar()
        self.end_vision()
        self.end_gui_server()





    def main_show_vision_data(self):
        '''Show vision data'''
        self.main_show_vision_data_run = True
        while self.main_show_vision_data_run:
            try:
                print('status : {} | x : {} | y : {} | theta : {} | Use Ctrl+C or enter any key to end current process : '\
                    .format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_theta))
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
        self.main_run = False
    def _exit_l(self):
        self.end_lidar()
    def _exit_v(self):
        self.end_vision()




    ################ LiDAR ###############
    def _li(self):
        if not self.LI.lidar_run:
            self.lidar_init()
        else:
            print('LiDAR run already')
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
        print('status : {} | x : {} | y : {} | theta : {} '.format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_theta))
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
        self.AS.attitude[0] = int(self.VI.vision_theta)

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
