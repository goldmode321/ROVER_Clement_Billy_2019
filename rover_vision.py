import time
import xmlrpc.client as xmlrpclib
import numpy as np
import traceback
import threading
import logging
import math

class Vision:
    '''
    The module for controlling Vision module, communicate with
    bridge, ip should be specify as the fix ip of vision module
    '''
    def __init__(self, SharedVariables, ip=None):
        self.SV = SharedVariables
        self.VI = self.SV.VI
        self.COM = self.SV.COM
        self.COM.command_vision["vs"] = lambda: print('Vision server : {}\n'.format(self.VI.vision_run))
        self.COM.command_vision["gs"] = self.show_vision_status
        self.COM.command_vision["sv"] = self.save
        self.COM.command_vision["al"] = self.alive
        self.COM.command_vision["vcc"] = self.check_cpu_speed
        self.COM.command_vision["vrs"] = self.reset
        self.COM.command_vision["gp"] = lambda: print('status : {} | x : {} | y : {} | theta : {} '.format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_angle))
        self.COM.command_vision["gp start"] = self.continuous_show_vision_data
        self.COM.command_vision["gp stop"] = self.stop_show_vision_data
        self.COM.command_vision["bm"] = self.build_map
        self.COM.command_vision["um"] = self.use_map
        self.COM.command_vision["gbm"] = self.gui_build_map
        self.COM.command_vision["gum"] = self.gui_um
        self.COM.command_vision["gbms"] = self.gui_build_map_stop
        self.COM.command_vision["gums"] = self.gui_um_stop

        if ip is not None:
            self.VI.vision_ip = ip
        self.init()

    def init(self):
        ''' Initialize communication with vision module and tcn_bridge'''
        try:
            time.sleep(0.2) # Make sure server initialize first
            logging.basicConfig(filename='Vision_main.log', filemode='w', level=logging.INFO)
            # self.vision = self.VI.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.VI.vision_ip))
            self.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.VI.vision_ip))
            if self.vision.alive() == [0, 'Alive']:
                logging.info('Connection to Vision module establiished , Vision module status : {}'.format(self.vision.alive()))
                self.VI.vision_run = True
                self.start_background_thread()
            else:
                logging.info('Vision module is not Alive')
                raise KeyboardInterrupt
        except:
            self.VI.vision_run = False
            traceback.print_exc()
            logging.exception('Got error : ')



    def start_background_thread(self):
        self.vision_thread = VisionGetDataThread(self.SV, vision=self.vision)
        print('Vision Thread Start')


    def end(self):
        self.VI.vision_run = False
        if not self.VI.vision_idle:
            time.sleep(0.2)
            self.reset()
        time.sleep(0.5)
        logging.info("'exit' command received, terminating program")

    def alive(self):
        alive_resp = self.vision.alive()
        print('\nalive(), response: {}'.format(alive_resp))

    def check_cpu_speed(self):
        cc_resp = self.vision.check_cpu_speed()
        print('\nget_att(), response: {}'.format(cc_resp))

    def get_pose(self):
        pose_resp = self.vision.get_pose()
        print('\nget_pose(), response: {}'.format(pose_resp))

    def get_status(self):
        status_resp = self.vision.get_status()
        print('\nget_status(), response: {}'.format(status_resp))

    def save(self):
        print("Vision do save process and reset, please wait")
        self.reset_flag = False
        save_resp = self.vision.save_db()
        time.sleep(3)
        print('\nsave_db(), response: {}'.format(save_resp))

    def reset(self):
        self.VI.reset_flag = True
        self.VI.vision_run = False
        time.sleep(0.2)
        reset_resp = self.vision.reset()
        print('\nreset(), response: {}'.format(reset_resp))
        print("Please wait for 10 seconds")
        time.sleep(10)
        self.VI.vision_run = True
        self.VI.vision_build_map_mode = False
        self.VI.vision_use_map_mode = False
        self.start_background_thread()

    def build_map(self): # Build map
        try:
            self.VI.vision_map_id = int(input('MapID : '))
            if self.VI.vision_map_id is not None:
                start_resp = self.vision.set_start(1, [self.VI.vision_map_id])
                print('\nset_start(), response: {}'.format(start_resp))
                self.VI.vision_build_map_mode = True
                time.sleep(2)
                print('\nset_correct : {}'.format(self.vision.set_correct1([100, 1352, 1352, 1352, 1352])))
                try:
                    input("Use Ctrl+C or enter any key to end current process : ")
                    self.save()
                    self.reset()
                except KeyboardInterrupt:
                    self.save()
                    self.reset()
                self.VI.vision_build_map_mode = False
            else:
                print("\n'Build map' command received , but no mapid")
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')

    def use_map(self): # Use map
        try:
            self.VI.vision_map_id = int(input('MapID : '))
            if self.VI.vision_map_id is not None:
                start_resp = self.vision.set_start(0, [self.VI.vision_map_id])
                print('\nset_start(), response: {}'.format(start_resp))
                self.VI.vision_use_map_mode = True
                try:
                    input("Use Ctrl+C or enter any key to end current process : ")
                    self.save()
                    self.reset()
                except KeyboardInterrupt:
                    self.save()
                    self.reset()
                self.VI.vision_use_map_mode = False
            else:
                print("\n'Use map' command received , but no mapid")
        except ValueError:
            print('Please specify MapID in integer')
        except KeyboardInterrupt:
            print('Abort')

    def gui_build_map(self):
        self.VI.vision_map_id = self.COM.gui_command_receive[1]
        if self.VI.vision_map_id is not None:
            start_resp = self.vision.set_start(1, [self.VI.vision_map_id])
            self.VI.vision_build_map_mode = True
            print('\nset_start(), response: {}'.format(start_resp))
            time.sleep(2)
            print('\nset_correct : {}'.format(self.vision.set_correct1([100, 1352, 1352, 1352, 1352])))
        else:
            print("\n'Build map' command received , but no mapid")

    def gui_build_map_stop(self):
        print("Vision save, please wait")
        self.save()
        print("Vision reset, please wait")
        self.reset()
        print("Vision is now idling")
        self.VI.vision_build_map_mode = False

    def gui_um(self):
        self.VI.vision_map_id = self.COM.gui_command_receive[1]
        if self.VI.vision_map_id is not None:
            start_resp = self.vision.set_start(0, [self.VI.vision_map_id])
            self.VI.vision_build_map_mode = True
            print('\nset_start(), response: {}'.format(start_resp))
            time.sleep(2)
        else:
            print("\n'Build map' command received , but no mapid")

    def gui_um_stop(self):
        print("Vision reset, please wait")
        self.reset()
        self.VI.vision_use_map_mode = False


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

    def continuous_show_vision_data(self):
        '''Show vision data'''
        self.continuous_show_vision_data_run = True
        while self.continuous_show_vision_data_run:
            try:
                print('status : {} | x : {} | y : {} | theta : {} | Use Ctrl+C or enter any key to end current process : '\
                    .format(self.VI.vision_status, self.VI.vision_x, self.VI.vision_y, self.VI.vision_angle))
                time.sleep(0.1)
            except:
                print('\nError from Main : main_send_vision_data_to_commander\n')
                traceback.print_exc()
                logging.exception('Got error : ')
                self.continuous_show_vision_data_run = False

    def stop_show_vision_data(self):
        self.continuous_show_vision_data_run = False

class VisionGetDataThread(threading.Thread):
    def __init__(self, SharedVariableS, vision, daemon=True):
        self.SV = SharedVariableS
        self.VI = self.SV.VI
        self.CAL = self.SV.CAL
        self.vision = vision
        threading.Thread.__init__(self, daemon=daemon)
        self.start()


    def run(self):
        '''Send vision data to bridge'''
        time.sleep(0.1)
        while self.VI.vision_run:
            try:
                if self.VI.reset_flag:
                    self.VI.reset_flag = False
                    pass
                else:
                    status = self.vision.get_status()
                    pose = self.vision.get_pose()
                    self.VI.vision_status = status[0]
                    self.VI.vision_angle = pose[5] * self.CAL.calibrate_angle_multi + self.CAL.calibrate_angle
                    self.VI.vision_angle_radian = math.radians(self.VI.vision_angle)
                    self.VI.vision_x = round((pose[3]-self.CAL.calibrate_dis_lv*np.sin(self.VI.vision_angle_radian))* \
                        self.CAL.calibrate_x_multi + self.CAL.calibrate_x, 1)
                    self.VI.vision_y = round((pose[4]-self.CAL.calibrate_dis_lv*np.cos(self.VI.vision_angle_radian))* \
                        self.CAL.calibrate_y_multi + self.CAL.calibrate_y, 1)
                    if self.VI.vision_status == 1:
                        self.VI.vision_idle = True
                    else:
                        self.VI.vision_idle = False

                time.sleep(0.15)
            except:
                logging.exception('Vision thread got error : ')
                print("Vision thread stop due to error")
                self.VI.vision_run = False

