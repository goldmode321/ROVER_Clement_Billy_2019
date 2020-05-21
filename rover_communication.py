import time
import traceback
import threading
import numpy as np

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

        # Vision initial variables
        self.VI.vision_build_map_mode = False
        self.VI.vision_use_map_mode = False

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
            "gss": self._gui_set_speed, "gkcc":self._gui_keyboard_control,
            "gcal":self._gui_cal, "gsvgobs":self._gui_svgobs, "gimport":self._gui_import,
            "glmm":self._gui_lmm, "gptg":self._gui_pt_gain, "gppe":self._gui_pp_e,
            "gppr":self._gui_pp_r,

        }

        self.thread_gui = threading.Thread(target=self.guiSendAndRead, daemon=True)
        self.thread_gui.start()
        self.thread_gui_get_command = threading.Thread(target=self.guiGetCommand, daemon=True)
        self.thread_gui_get_command.start()


    def guiGetCommand(self):
        while self.gui_server_run:
            try:
                self.COM.gui_command_receive = self.gui_command_server.recv_list()
                if self.COM.gui_command_receive is not None:
                    if self.COM.gui_command_receive[0] in self.gui_command:
                        self.gui_command[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_rover:
                        self.COM.command_rover[self.COM.gui_command_receive[0]]() # Referenced from CommanderDictionary
                    elif self.COM.gui_command_receive[0] in self.COM.command_vision:
                        self.COM.command_vision[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_lidar:
                        self.COM.command_lidar[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_car_control:
                        self.COM.command_car_control[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_path_planning:
                        self.COM.command_path_planning[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_path_tracking:
                        self.COM.command_path_tracking[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_map_builder:
                        self.COM.command_map_builder[self.COM.gui_command_receive[0]]()
                    elif self.COM.gui_command_receive[0] in self.COM.command_calibration:
                        self.COM.command_calibration[self.COM.gui_command_receive[0]]()
                    else:
                        print('Unknown Command {}'.format(self.COM.gui_command_receive))
                    # print('\n GUI Command {}'.format(self.COM.gui_command_receive))
            except:
                traceback.print_exc()
                print("Something Wrong, maybe wrong protocol : {}".format(self.COM.gui_command_receive))
            time.sleep(0.05)



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

    def _gui_cal(self):
        self.CAL.calibrate_x = self.COM.gui_command_receive[1]
        self.CAL.calibrate_x_multi = self.COM.gui_command_receive[2]
        self.CAL.calibrate_y = self.COM.gui_command_receive[3]
        self.CAL.calibrate_y_multi = self.COM.gui_command_receive[4]
        self.CAL.calibrate_angle = self.COM.gui_command_receive[5]
        self.CAL.calibrate_angle_multi = self.COM.gui_command_receive[6]
        self.CAL.calibrate_dis_lv = self.COM.gui_command_receive[7]

    def _gui_keyboard_control(self):
        self.CC.car_control_move = self.COM.gui_command_receive[1]
        self.CC.car_control_steer = self.COM.gui_command_receive[2]

    def _gui_set_speed(self):
        self.CC.car_control_add_speed = self.COM.gui_command_receive[1]

    def _gui_import(self):
        try:
            if self.COM.gui_command_receive[1] != "":
                data = np.load("./map/{}".format(self.COM.gui_command_receive[1]))
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
                print("Import map {} complete".format(self.COM.gui_command_receive[1]))
        except:
            traceback.print_exc()

    def _gui_lmm(self):
        self.LI.lidar_maximum_radius = self.COM.gui_command_receive[1]
        self.LI.lidar_minimum_radius = self.COM.gui_command_receive[2]

    def _gui_svgobs(self):
        if self.COM.gui_command_receive[1] != "":
            np.savez(
                self.COM.gui_command_receive[1], 
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
            print("global map save to {}".format(self.COM.gui_command_receive[1]))
        print("Abort save process")

    def _gui_pt_gain(self):
        self.PT.theta_e_gain = self.COM.gui_command_receive[1]
        self.PT.theta_d_gain = self.COM.gui_command_receive[2]

    def _gui_pp_e(self):
        self.AS.end_x = self.COM.gui_command_receive[1]
        self.AS.end_y = self.COM.gui_command_receive[2]

    def _gui_pp_r(self):
        self.AS.rover_size = self.COM.gui_command_receive[1]
        self.AS.step_unit = self.COM.gui_command_receive[2]





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