import time
import xmlrpc.client as xmlrpclib
import traceback
import threading
import logging
import math

class Vision:
    '''
    The module for controlling Vision module, communicate with
    bridge, ip should be specify as the fix ip of vision module
    '''
    def __init__(self, SharedVariable_vision, auto_start=True, ip=None):
        self.VI = SharedVariable_vision
        if ip is not None:
            self.VI.vision_ip = ip

        if auto_start:
            self.autorun()

    def autorun(self):
        ''' Auto start step'''
        self.init()
        self.start_background_thread()


    def init(self):
        ''' Initialize communication with vision module and tcn_bridge'''
        try:
            time.sleep(0.2) # Make sure server initialize first
            logging.basicConfig(filename='Vision_main.log', filemode='w', level=logging.INFO)
            # self.vision = self.VI.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.VI.vision_ip))
            self.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.VI.vision_ip))
            if self.vision.alive() == [0, 'Alive']:
                logging.info('Connection to Vision module establiished , Vision module status : {}\n'.format(self.vision.alive()))
                self.VI.vision_run = True
                self.start_background_thread()
            else:
                logging.info('Vision module is not Alive\n')
                raise KeyboardInterrupt
        except:
            self.VI.vision_run = False
            traceback.print_exc()
            logging.exception('Got error : ')



    def start_background_thread(self):
        self.vision_thread = VisionGetDataThread(self.VI, vision=self.vision)
        logging.info('Thread running')


    def end(self):
        self.VI.vision_run = False
        if not self.VI.vision_idle:
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
        self.start_background_thread()

    def build_map(self, mapid): # Build map
        if mapid is not None:
            start_resp = self.vision.set_start(1, [mapid])
            print('\nset_start(), response: {}'.format(start_resp))
            logging.info("'Build map' command received , mapid : %s ", mapid)
        else:
            print("\n'Build map' command received , but no mapid")
            logging.info("'Build map' command received , but no mapid")

    def use_map(self, mapid): # Use map
        if mapid is not None:
            start_resp = self.vision.set_start(0, [mapid])
            print('\nset_start(), response: {}'.format(start_resp))
            logging.info("'Use map' command received , mapid : %s ", mapid)
        else:
            print("\n'Use map' command received , but no mapid")
            logging.info("'Use map' command received , but no mapid")


class VisionGetDataThread(threading.Thread):
    def __init__(self, SharedVariable_vision, vision, daemon=True):
        self.VI = SharedVariable_vision
        self.vision = vision
        threading.Thread.__init__(self, daemon=daemon)
        self.start()


    def run(self):
        '''Send vision data to bridge'''
        while self.VI.vision_run:
            try:
                # print('v')
                if self.VI.reset_flag:
                    # time.sleep(7)
                    self.VI.reset_flag = False
                    # time.sleep(1)
                    pass
                else:
                    status = self.vision.get_status()
                    pose = self.vision.get_pose()
                    self.VI.vision_status = status[0]
                    self.VI.vision_x = pose[3]
                    self.VI.vision_y = pose[4]
                    self.VI.vision_theta = pose[5]
                    self.VI.vision_angle_radian = math.radians(pose[5])
                    if self.VI.vision_status == 1:
                        self.VI.vision_idle = True
                    else:
                        self.VI.vision_idle = False

                time.sleep(0.15)
            except:
                logging.exception('Vision thread got error : ')

