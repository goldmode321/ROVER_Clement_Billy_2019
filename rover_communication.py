import time
import traceback
import threading

import rover_socket

class Communication():
    '''Communication center Main(auto_start=True)'''
    def __init__(self, SharedVariables):
        '''If auto_start is False, program automatically run __init__ only'''

        self.SV = SharedVariables
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

        self.vision = self.COM.vision
        self.lidar = self.COM.lidar







        # Vision initial variables
        self.vision_build_map_mode = False
        self.vision_use_map_mode = False

        # GUI initial variables
        self.gui_server_run = False
        self.gui_receive = []


        self.gui_test_connection_server = rover_socket.UDP_server(50011, ip=self.ROV.rover_ip)
        self.gui_command_server = rover_socket.UDP_server(50012, ip=self.ROV.rover_ip)
        self.gui_rov_server = rover_socket.UDP_server(50013, ip=self.ROV.rover_ip)
        self.gui_vi_server = rover_socket.UDP_server(50014, ip=self.ROV.rover_ip)
        self.gui_li_server = rover_socket.UDP_server(50015, ip=self.ROV.rover_ip)
        self.gui_cal_server = rover_socket.UDP_server(50016, ip=self.ROV.rover_ip)
        self.gui_cc_server = rover_socket.UDP_server(50017, ip=self.ROV.rover_ip)
        self.gui_lobs_server = rover_socket.UDP_server(50018, ip=self.ROV.rover_ip)
        self.gui_gobs_server = rover_socket.UDP_server(50019, ip=self.ROV.rover_ip)
        self.gui_as_server = rover_socket.UDP_server(50020, ip=self.ROV.rover_ip)
        self.gui_cf_server = rover_socket.UDP_server(50021, ip=self.ROV.rover_ip)
        self.gui_pt_server = rover_socket.UDP_server(50022, ip=self.ROV.rover_ip)

        self.gui_server_run = True
        self.gui_command = {
            "gss": self._gui_set_speed, "gbm": self._gui_bm, "gum":self._gui_um, \
            "gbms": self._gui_bm_stop, "gums":self._gui_um_stop, "gkcc":self._gui_keyboard_control
        }

        self.thread_gui = threading.Thread(target=self.guiSendAndRead, daemon=True)
        self.thread_gui.start()
        self.thread_gui_get_command = threading.Thread(target=self.guiGetCommand, daemon=True)
        self.thread_gui_get_command.start()


    def guiGetCommand(self):
        while self.gui_server_run:

            self.gui_command_receive = self.gui_test_connection_server.recv_list()
            if self.gui_command_receive is not None:
                if self.gui_command_receive[0] in self.gui_command:
                    self.gui_command[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_rover:
                    self.COM.command_rover[self.gui_command_receive[0]]() # Referenced from CommanderDictionary
                elif self.gui_command_receive[0] in self.COM.command_vision:
                    self.COM.command_vision[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_lidar:
                    self.COM.command_lidar[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_car_control:
                    self.COM.command_car_control[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_path_planning:
                    self.COM.command_path_planning[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_path_tracking:
                    self.COM.command_path_tracking[self.gui_command_receive[0]]()
                elif self.gui_command_receive[0] in self.COM.command_map_builder:
                    self.COM.command_map_builder[self.gui_command_receive[0]]()
                else:
                    print('Unknown Command')
            time.sleep(0.1)



    def guiSendAndRead(self):
        while self.gui_server_run:
            test_connection = self.gui_test_connection_server.recv_object(64)
            self.gui_rov_server.recv_object(16)
            self.gui_vi_server.recv_object(16)
            self.gui_li_server.recv_object(16)
            self.gui_cal_server.recv_object(16)
            self.gui_cc_server.recv_object(16)
            self.gui_lobs_server.recv_object(16)
            self.gui_gobs_server.recv_object(16)
            self.gui_as_server.recv_object(16)
            self.gui_cf_server.recv_object(16)
            self.gui_pt_server.recv_object(16)

            if test_connection is not None:
                self.gui_test_connection_server.send_object_back(1)
                self.gui_rov_server.send_object_back(self.ROV)
                self.gui_vi_server.send_object_back(self.VI)
                self.gui_li_server.send_object_back(self.LI)
                self.gui_cal_server.send_object_back(self.CAL)
                self.gui_cc_server.send_object_back(self.CC)
                self.gui_lobs_server.send_object_back(self.LOBS)
                self.gui_gobs_server.send_object_back(self.GOBS)
                self.gui_as_server.send_object_back(self.AS)
                self.gui_cf_server.send_object_back(self.CF)
                self.gui_pt_server.send_object_back(self.PT)

            time.sleep(0.1)



    def end_gui_server(self):
        self.gui_server_run = False
        self.thread_gui.join()
        self.thread_gui_get_command.join()
        self.gui_command_server.close()
        self.gui_test_connection_server.close()
        self.gui_rov_server.close()
        self.gui_vi_server.close()
        self.gui_li_server.close()
        self.gui_cal_server.close()
        self.gui_cc_server.close()
        self.gui_lobs_server.close()
        self.gui_gobs_server.close()
        self.gui_as_server.close()
        self.gui_cf_server.close()
        self.gui_pt_server.close()


    def _gui_keyboard_control(self):
        self.CC.car_control_move = self.gui_command_receive[1]
        self.CC.car_control_steer = self.gui_command_receive[2]

    def _gui_set_speed(self):
        self.CC.car_control_add_speed = self.gui_command_receive[1]
    def _gui_bm(self):
        # self.vision_server.send_list(['V', 'bm', self.gui_command_receive[1]])
        self.vision.build_map(self.gui_command_receive[1])
        # self.vision_idle = False
        self.vision_build_map_mode = True
    def _gui_bm_stop(self):
        print("Vision save, please wait")
        # self.vision_server.send_list(['V', 'sv'])
        self.vision.save()
        print("Vision reset, please wait")
        # self.vision_server.send_list(['V', 'rs'])
        self.vision.reset()

        print("Vision is now idling")
        # self.vision_idle = True
        self.vision_build_map_mode = False
    def _gui_um(self):
        # self.vision_server.send_list(['V', 'um', self.gui_command_receive[1]])
        self.vision.use_map(self.gui_command_receive[1])
        # self.vision_idle = False
        self.vision_use_map_mode = True
    def _gui_um_stop(self):
        print("Vision reset, please wait")
        # self.vision_server.send_list(['V', 'rs'])
        self.vision.reset()

        # self.vision_idle = True
        self.vision_use_map_mode = False


            # self.gui_rov_server
            # self.gui_vi_server
            # self.gui_li_server
            # self.gui_cal_server
            # self.gui_cc_server
            # self.gui_lobs_server
            # self.gui_gobs_server
            # self.gui_as_server
            # self.gui_cf_server
            # self.gui_pt_server