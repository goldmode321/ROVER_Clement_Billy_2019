import time
import traceback
import subprocess
import threading
import multiprocessing
import logging
import Adafruit_PCA9685
import rover_socket

class MoveAlgorithm:
    ''' Provide different moving mode '''
    def __init__(self, vision_status, vision_data, lidar_data, recorded_vision_coordinate):
        self.vision_data = vision_data
        self.vision_status = vision_status
        self.lidar_data = lidar_data
        self.recorded_vision_coordinate = recorded_vision_coordinate
        self.global_obstacle = []

    def algorithm_move_between_waypoints(self):
        pass

    def algorithm_get_global_obstacle(self):
        if self.vision_status != 4:
            pass

class CarControl(threading.Thread):
    def __init__(self):
        self.car_control_server = rover_socket.UDP_server(50011, 0, "192.168.5.2")
        self.car_control = Adafruit_PCA9685.PCA9685()
        self.car_control.set_pwm_freq(60)
        self.car_control_server_run = True
        self.car_control_receive = None
        self.car_control_previous_receive = None
        self.car_control_state = 'stop'
        self.car_control_forward_pwm = 410 # 403
        self.car_control_backward_pwm = 370  # 381
        self.car_control_stop_pwm = 400
        self.car_control_add_speed = 1 # Speed adjust from gui
        threading.Thread.__init__(self, daemon=True)
        thread = threading.Thread(target=self.car_control_main, daemon=True)
        thread.start()

    def car_control_main(self):
        while self.car_control_server_run:

            temp = self.car_control_server.recv_list()
            # print(temp)
            if temp is None and self.car_control_previous_receive is not None:
                self.car_control_receive = self.car_control_previous_receive
                self.car_control_previous_receive = None
                # print('is none')
                # time.sleep(0.05)
            elif temp is None and self.car_control_previous_receive is None:
                self.car_control_receive = None
            else:
                self.car_control_previous_receive = self.car_control_receive
                self.car_control_receive = temp

            if self.car_control_receive is not None :
                if self.car_control_receive[0] in ['a', 'wa', 'sa']:
                    self.turn_left()
                elif self.car_control_receive[0] in ['d', 'wd', 'sd']:
                    self.turn_right()
                else:
                    self.straight()


            else:
                # time.sleep(0.05)
                self.straight()
                # self.car_control_previous_receive = self.car_control_receive
            # time.sleep(0.15)
            # time.sleep(0.05)

    def run(self):
        while self.car_control_server_run:
            # print(car_control_receive)
            # print(self.car_control_receive)
            if self.car_control_receive is not None:
                self.car_control_protocol(self.car_control_receive[0])
            else:
                self.stop()
            # if self.car_control_server.addr is not None:
            #     self.car_control_server.send_list_back([self.car_control_state])
                # time.sleep(0.1)
            # if self.car_control_server.addr is not None:
            #     self.car_control_server.send_string_back('next')

    def end_car_control(self):
        self.car_control_server_run = False
        self.car_control_server.close()

    def car_control_protocol(self, car_control_receive):
        if car_control_receive is not None:
            if car_control_receive in ['w', 'wa', 'wd']:
                if self.car_control_state == 'stop':
                    self.forward()
                    self.car_control_state = 'forward'
                elif self.car_control_state == 'reverse':
                    self.forward_after_reverse()
                    self.car_control_state = 'forward'
                elif self.car_control_state == 'forward':
                    self.forward()

            elif car_control_receive in ['s', 'sd', 'sa']:
                if self.car_control_state == 'stop':
                    self.reverse_after_forward()
                    # keep_reversing_after_reverse()
                    self.car_control_state = 'reverse'
                elif self.car_control_state == 'forward':
                    # self.reverse()
                    self.reverse_after_forward()
                    self.car_control_state = 'reverse'
                elif self.car_control_state == 'reverse':
                    self.reverse()
            else:
                self.stop()
                self.car_control_state = 'stop'


        else:
            self.stop()
            self.car_control_state = 'stop'
            # self.straight()


    def forward_after_reverse(self):
        self.car_control.set_pwm(3,0,self.car_control_stop_pwm)
        time.sleep(0.1)
        self.car_control.set_pwm(3,0,self.car_control_forward_pwm + self.car_control_add_speed)
        time.sleep(0.05)

        # self.car_control.set_pwm(3,0,415)
        # time.sleep(0.1)
        # self.car_control.set_pwm(3,0,400)
        # time.sleep(0.1)
        # self.car_control.set_pwm(3,0,415)
        # time.sleep(0.1)
    def reverse_after_forward(self):
        self.car_control.set_pwm(3,0,self.car_control_stop_pwm)
        time.sleep(0.1)
        self.car_control.set_pwm(3,0,self.car_control_backward_pwm - self.car_control_add_speed)
        time.sleep(0.05)

        # self.car_control.set_pwm(3,0,350)
        # time.sleep(0.1)
        # self.car_control.set_pwm(3,0,400)
        # time.sleep(0.1)
        # self.car_control.set_pwm(3,0,383)
        # time.sleep(0.1)
    def stop(self):
        self.car_control.set_pwm(3,0,self.car_control_stop_pwm)
        time.sleep(0.05)

        # self.car_control.set_pwm(3,0,400)
        # time.sleep(0.15)
        # time.sleep(0.05)

    def forward(self):
        self.car_control.set_pwm(3, 0, self.car_control_forward_pwm + self.car_control_add_speed)
        time.sleep(0.05)

        # self.car_control.set_pwm(3, 0, 413)
        # time.sleep(0.05)
    def reverse(self):
        self.car_control.set_pwm(3, 0, self.car_control_backward_pwm - self.car_control_add_speed)
        time.sleep(0.05)

        # self.car_control.set_pwm(3, 0, 383)
        # time.sleep(0.05)

    def turn_left(self):
        self.car_control.set_pwm(1,0,495)
        # time.sleep(0.15)
        time.sleep(0.05)

    def turn_right(self):
        self.car_control.set_pwm(1,0,315)
        # time.sleep(0.15)
        time.sleep(0.05)

    def straight(self):
        self.car_control.set_pwm(1,0,405)
        # time.sleep(0.15)
        time.sleep(0.05)

class Main(CarControl, MoveAlgorithm):
    '''Communication center Main(auto_start=True)'''
    def __init__(self, auto_start=True):
        '''If auto_start is False, program automatically run __init__ only'''
        logging.basicConfig(filename='Main.log', filemode='w', level=logging.INFO)
        # self.car_control = CarControl()
        # self.car_control.__init__()
        CarControl.__init__(self)

        # Main initial variables
        self.auto_start = auto_start
        self.main_run = False
        self.main_receive = []



        # Commander initail variables
        self.commander_server_run = False

        # Vision initial variables
        self.vision_x = 0
        self.vision_y = 0
        self.theta = 0
        self.vision_status = 0
        self.vision_data = [0, 0, 0, 0, 0]
        self.recorded_vision_coordinate = []
        self.vision_record_coordinate = False
        self.vision_client_run = False
        self.vision_server_run = False
        self.vision_thread_server_run = False
        self.vision_thread_server_status = 0
        self.vision_thread_client_run = False
        self.main_show_vision_data_run = False
        self.vision_thread = None
        self.vision_idle = False
        self.vision_build_map_mode = False
        self.vision_use_map_mode = False


        # Lidar initial variables
        self.lidar_data = []
        self.lidar_server_run = False
        self.lidar_thread_server_run = False
        self.lidar_thread_server_status = 0
        self.lidar_usb_port = ""
        self.lidar_state = [""]
        self.lidar_client_run = False
        self.lidar_thread = None

        # Algorithm initial variables
        self.algorithm_run = False
        self.algorithm_server_run = False

        # GUI initial variables
        self.gui_server_run = False
        self.gui_receive = []


        self.command_dictionary = {'exit all':self._exit_all, 'next':self._next}

        self.command_lidar_dictionary = {'exit l':self._exit_l, 'li':self._li, 'gld':self._gld, 'next':self._next}

        self.command_vision_dictionary = {'exit v':self._exit_v, 'vi':self._vi, 'vs':self._vs,\
                  'gs':self._gs, 'al':self._al, 'cc':self._cc, 'sv':self._sv, 'vrs':self._vrs, 'gp c':self._gp_c, \
                        'gp exit':self._gp_exit, 'gp':self._gp, 'bm':self._bm, 'um':self._um, 'next':self._next}

        # Inherit MoveAlgorithm
        # super.__init__(self.vision_data, self.lidar_data, self.recorded_vision_coordinate)

        if self.auto_start:


            self.vision_init()
            self.lidar_init()
            self.gui_connection_init()
            CarControl.start(self)
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
                else:
                    print('Unknown Command')
            self.gui_get_command_udp_server.close()


    def gui_send_and_read(self):
        while self.gui_server_run:
            self.gui_receive = self.gui_udp_client.recv_list()
            self.gui_send_status_receive = self.gui_send_status_udp_server.recv_list()
            if self.gui_receive is not None:
                self.gui_udp_client.send_list_back([self.lidar_data, self.vision_data[0:4]])
            if self.gui_send_status_receive is not None:
                self.gui_send_status_udp_server.send_list_back([self.lidar_usb_port, self.lidar_state, self.lidar_client_run, \
                    self.vision_status, self.vision_server_run, self.vision_idle, self.main_run, self.car_control_add_speed, \
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
        self.car_control_add_speed = self.gui_get_command_receive[1]
    def _gui_bm(self):
        self.vision_server.send_list(['V', 'bm', self.gui_get_command_receive[1]])
        self.vision_idle = False
        self.vision_build_map_mode = True
    def _gui_bm_stop(self):
        print("Vision save, please wait")
        self.vision_server.send_list(['V', 'sv'])
        self.main_receive = self.vision_server.recv_list()
        print("Vision reset, please wait")
        self.vision_server.send_list(['V', 'rs'])
        self.main_receive = self.vision_server.recv_list()
        print("Vision is now idling")
        self.vision_idle = True
        self.vision_build_map_mode = False
    def _gui_um(self):
        self.vision_server.send_list(['V', 'um', self.gui_get_command_receive[1]])
        self.vision_idle = False
        self.vision_use_map_mode = True
    def _gui_um_stop(self):
        print("Vision reset, please wait")
        self.vision_server.send_list(['V', 'rs'])
        self.main_receive = self.vision_server.recv_list()
        self.vision_idle = True
        self.vision_use_map_mode = False


################## Vision ##############

    def vision_init(self):
        '''Initialize vision system and communication '''
        try:
            logging.info("Initialize vision server\n")
            subprocess.Popen('python3 rover_vision.py', shell=True, start_new_session=True)
            self.vision_thread_server = rover_socket.UDP_server(50003)
            self.vision_server = rover_socket.TCP_server(50002)
            receive = self.vision_server.recv_list()
            if receive == ['V', 'next']:
                logging.info("Vision communication successfully established !\
                    \ncommunication center get : {} \n".format(receive))
                self.vision_server_run = True
                self.vision_thread_server_run = True
                self.vision_idle = True
                self.vision_start_background_thread()
            else:
                self.end_vision_server()
                self.end_vision_thread_server()
                print('{} received from TCN_vision'.format(receive))
                print('Either vision module got problem or undefined communication error of\
                     Vision module, please check test message')
                logging.info('{} received from TCN_vision'.format(receive))
                logging.info("Either vision module got problem or undefined communication \
                    error of Vision module, please check test message\n")
        except:
            self.vision_server.close()
            self.vision_thread_server.close()
            print('\nError from Main : vision_init \n')
            traceback.print_exc()
            logging.info('Main initializing fail at vision_init()\n')
            logging.exception("Got error : \n")

    def vision_start_background_thread(self):
        '''Backgound thread that get store incoming data'''
        self.vision_thread = threading.Thread(target=self.vision_thread_main, daemon=True)
        self.vision_thread.start()
        logging.info('Vision thread start')

    def vision_thread_main(self):
        '''Get vision data'''
        while self.vision_thread_server_run:
            self.vision_thread_server_status = self.vision_thread.is_alive()
            temp_vision_data_received = self.vision_thread_server.recv_list()
            if temp_vision_data_received is not None:
                self.vision_data = temp_vision_data_received
                self.vision_x = self.vision_data[0]
                self.vision_y = self.vision_data[1]
                self.theta = self.vision_data[2]
                self.vision_status = self.vision_data[3]
                self.vision_client_run = self.vision_data[4]
                self.vision_thread_client_run = self.vision_data[5]
            else:
                pass
            time.sleep(0.1)

    def end_vision_server(self):
        '''End vision module system'''
        if self.vision_server_run:
            self.vision_server.send_list(['V', 'exit'])
            time.sleep(1)
            self.vision_server.close()
            self.vision_server_run = False
            logging.info('Vision server end')
        else:
            print('Vision TCP server already off')

    def end_vision_thread_server(self):
        '''Stop getting data from vision module'''
        if self.vision_thread_server_run:
            self.vision_thread_server.close()
            if self.vision_thread_server_run:
                self.vision_thread_server_run = False
                self.vision_thread.join()
                self.vision_thread_server_status = self.vision_thread.is_alive()
            logging.info('Vision thread stop')


################## LiDAR ######################

    def lidar_init(self):
        '''Initialize Lidar system and communication'''
        try:
            logging.info("Initialize lidar server\n")
            subprocess.Popen('python3 rover_rplidar.py', shell=True, start_new_session=True)
            self.lidar_server = rover_socket.TCP_server(50004)
            self.lidar_thread_server = rover_socket.UDP_server(50005)
            lidar_data = self.lidar_server.recv_list()
            if lidar_data == ['L', 'status', 'Good']:
                logging.info("Lidar communication successfully established !\ncommunication center get : {} \n".format(lidar_data))
                self.lidar_server_run = True
                self.lidar_thread_server_run = True
                self.lidar_start_background_thread()
            else:
                self.end_lidar_server()
                self.end_lidar_thread_server()
                print('Undefined communication error of LiDAR, please check test message')
                logging.info("Undefined communication error of Vision module, please check test message\n")
                raise KeyboardInterrupt
        except:
            print('\nError from Main : lidar_init\n')
            traceback.print_exc()
            logging.info('Main initializing fail at lidar_init()\n')
            logging.exception("Main initializing fail at lidar_init() : \n")
            self.lidar_server.close()
            self.lidar_thread_server.close()

    def lidar_start_background_thread(self):
        '''Lidar get data in background'''
        self.lidar_thread = threading.Thread(target=self.lidar_thread_main, daemon=True)
        self.lidar_thread.start()
        logging.info('LiDAR thread start')

    def lidar_thread_main(self):
        '''Lidar get data'''
        while self.lidar_thread_server_run:
            self.lidar_thread_server_status = self.lidar_thread.is_alive()
            if self.lidar_thread_server.server_alive:
                temp_lidar_data = self.lidar_thread_server.recv_list(65536)
                if temp_lidar_data is not None:
                    self.lidar_usb_port = temp_lidar_data[0]
                    self.lidar_data = temp_lidar_data[1]
                    self.lidar_state = temp_lidar_data[2]
                    self.lidar_client_run = temp_lidar_data[3]
            time.sleep(0.05)

    def end_lidar_server(self):
        '''End lidar system'''
        if self.lidar_server_run:
            self.lidar_server.send_list(['L', 'exit'])
            time.sleep(1)
            self.lidar_server.close()
            self.lidar_server_run = False
            logging.info('LiDAR server end')
        else:
            print('LiDAR TCP server already off')

    def end_lidar_thread_server(self):
        '''Stop storing lidar data'''
        if self.lidar_thread_server_run:
            self.lidar_thread_server.close()
            if self.lidar_thread_server_run:
                self.lidar_thread_server_run = False
                self.lidar_thread.join()
                self.lidar_thread_server_status = self.lidar_thread.is_alive()
            logging.info('LiDAR thread end')


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

        self.end_lidar_server()
        self.end_lidar_thread_server()
        self.end_vision_server()
        self.end_vision_thread_server()
        self.end_gui_server()
        self.end_car_control()




    def main_show_vision_data(self):
        '''Show vision data'''
        self.main_show_vision_data_run = True
        while self.main_show_vision_data_run:
            try:


                print('status : {} | x : {} | y : {} | theta : {} | Use Ctrl+C or enter any key to end current process : '\
                    .format(self.vision_status, self.vision_x, self.vision_y, self.theta))
                time.sleep(0.1)
            except:
                print('\nError from Main : main_send_vision_data_to_commander\n')
                traceback.print_exc()
                logging.exception('Got error : ')
                self.main_show_vision_data_run = False








    def show_vision_status(self):
        '''Show message depends on vision status'''
        if self.vision_status == 0:
            print("Vision module status : {} | Vision module is booting".format(self.vision_status))
        elif self.vision_status == 1:
            print("Vision module status : {} | Vision module is waiting for 'st $mapid' command".format(self.vision_status))
        elif self.vision_status == 2:
            print("Vision module status : {} | Vision module is loading data ".format(self.vision_status))
        elif self.vision_status == 3:
            print('Vision module status : {} | Please move slowly, fp-slam is searching a set of best images to initialize'.format(self.vision_status))
        elif self.vision_status == 4:
            print('Vision module status : {} | System is working normaaly'.format(self.vision_status))
        elif self.vision_status == 5:
            print('Vision module status : {} | Lost Lost Lost'.format(self.vision_status))
        else:
            print('Unknown status code : {}'.format(self.vision_status))






############ Command relative function ###########
    def _exit_all(self):
        self.end_main_all()
        self.main_run = False
    def _exit_b(self):
        self.end_main_all()
        self.main_run = False
    def _exit_l(self):
        self.end_lidar_server()
        self.end_lidar_thread_server()
    def _exit_v(self):
        self.end_vision_server()
        self.end_vision_thread_server()



    ################ LiDAR ###############
    def _li(self):
        if not self.lidar_server_run:
            self.lidar_init()
        else:
            print('LiDAR run already')
    def _gld(self):
        print(self.lidar_data)

    ################ Vision #############
    def _vi(self):
        if not self.vision_server_run:
            self.vision_init()
        else:
            print('Vision run already')
    def _vs(self):
        print('Vision server : {}\nVision thread server : {}\nVision client : {}\nVision thread client : {}'\
            .format(self.vision_server_run, self.vision_thread_server_run, self.vision_client_run, self.vision_thread_client_run))
    def _gs(self):
        self.show_vision_status()
    def _sv(self):
        self.vision_server.send_list(['V', 'sv'])
    def _al(self):
        self.vision_server.send_list(['V', 'al'])
    def _cc(self):
        self.vision_server.send_list(['V', 'cc'])
    def _vrs(self):
        self.vision_server.send_list(['V', 'rs'])
        print("Please wait for 10 seconds")
        self.vision_server.recv_list()
    def _gp_c(self):
        self.main_show_vision_data()
    def _gp_exit(self):
        self.main_show_vision_data_run = False
    def _gp(self):
        print('status : {} | x : {} | y : {} | theta : {} '.format(self.vision_status, self.vision_x, self.vision_y, self.theta))
    def _bm(self):
        try:
            mapid = int(input('MapID : '))
            self.vision_server.send_list(['V', 'bm', mapid])
            try:
                input("Use Ctrl+C or enter any key to end current process : ")
                print("Vision do save process and reset, please wait")
                self.vision_server.send_list(['V', 'sv'])
                self.main_receive = self.vision_server.recv_list()
                self.vision_server.send_list(['V', 'rs'])
                self.main_receive = self.vision_server.recv_list()
            except KeyboardInterrupt:
                print("Vision do save process and reset, please wait")
                self.vision_server.send_list(['V', 'sv'])
                self.main_receive = self.vision_server.recv_list()
                self.vision_server.send_list(['V', 'rs'])
                self.main_receive = self.vision_server.recv_list()
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')
    def _um(self):
        try:
            mapid = int(input('MapID : '))
            self.vision_server.send_list(['V', 'um', mapid])
            try:
                input("Use Ctrl+C or enter any key to end current process : ")
                print("Vision reset, please wait")
                self.vision_server.send_list(['V', 'rs'])
                self.main_receive = self.vision_server.recv_list()
            except KeyboardInterrupt:
                print("Vision reset, please wait")
                self.vision_server.send_list(['V', 'rs'])
                self.main_receive = self.vision_server.recv_list()
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')
    def _next(self):
        pass









if __name__ == "__main__":
    Main()
