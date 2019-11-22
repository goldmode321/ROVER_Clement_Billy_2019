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
            self.vision = self.VI.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.VI.vision_ip))
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
        self.VI.vision_thread = VisionGetDataThread(self.VI)
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
    def __init__(self, SharedVariable_vision, daemon=True):
        self.VI = SharedVariable_vision
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
                    status = self.VI.vision.get_status()
                    pose = self.VI.vision.get_pose()
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



# class Vision:
#     '''
#     The module for controlling Vision module, communicate with
#     bridge, ip should be specify as the fix ip of vision module
#     '''
#     def __init__(self, auto_start=True, ip='192.168.5.101'):
#         self.ip = ip
#         self.vision_client_run = False
#         self.vision_thread_client_run = False
#         self.reset_flag = False
#         self.vision_x = 0
#         self.vision_y = 0
#         self.vision_theta = 0
#         self.vision_status = 0
#         self.vision = None
#         self.vision_client = None
#         self.vision_thread_client = None

#         if auto_start:
#             self.autorun()

#     def autorun(self):
#         ''' Auto start step'''
#         self.init()
#         self.start_background_thread()
#         self.main()
#         self.end()
#         self.end_background_thread()

#     def init(self):
#         ''' Initialize communication with vision module and tcn_bridge'''
#         try:
#             time.sleep(0.2) # Make sure server initialize first
#             logging.basicConfig(filename='Vision_main.log', filemode='w', level=logging.INFO)
#             self.vision = xmlrpclib.ServerProxy("http://{}:8080".format(self.ip))
#             self.vision_thread_client = rover_socket.UDP_client(50003)
#             self.vision_client = rover_socket.TCP_client(50002)
#             if self.vision.alive() == [0, 'Alive']:
#                 logging.info('Connection to Vision module establiished , Vision module status : {}\n'.format(self.vision.alive()))
#                 self.vision_client.send_list(['V', 'next'])
#                 self.vision_client_run = True
#             else:
#                 logging.info('Vision module is not Alive\n')
#                 raise KeyboardInterrupt
#         except:
#             self.end()
#             self.end_background_thread()
#             traceback.print_exc()
#             logging.exception('Got error : ')

#     def main(self):
#         '''Process that wait for bridge command'''
#         while self.vision_client_run:
#             try:
#                 vision_receive = self.vision_client.recv_list()
#                 logging.info('Command in : {} '.format(vision_receive))
#                 self.reset_flag = True
#                 self.vision_portocol(vision_receive)
#             except:
#                 traceback.print_exc()
#                 self.vision_client_run = False
#                 self.vision_thread_client_run = False
#                 logging.exception('Got error : \n')
#                 self.end()
#                 self.end_background_thread()

#     def start_background_thread(self):
#         '''Start sending data thread'''
#         thread = threading.Thread(target=self.send_vision_status, daemon=True)
#         self.vision_thread_client_run = True
#         thread.start()
#         logging.info('Thread running')

#     def send_vision_status(self):
#         '''Send vision data to bridge'''
#         while self.vision_thread_client_run:
#             try:
#                 if self.reset_flag:
#                     # time.sleep(7)
#                     self.reset_flag = False
#                     # time.sleep(1)
#                 else:
#                     status = self.vision.get_status()
#                     pose = self.vision.get_pose()
#                     self.vision_status = status[0]
#                     self.vision_x = pose[3]
#                     self.vision_y = pose[4]
#                     self.vision_theta = pose[5]
#                     self.vision_thread_client.send_list([self.vision_x, self.vision_y, \
#                         self.vision_theta, self.vision_status, self.vision_client_run, \
#                             self.vision_thread_client_run])
#                 time.sleep(0.15)
#             except:
#                 logging.exception('Vision thread got error : ')



#     def vision_portocol(self, vision_receive):
#         '''Intepret message from bridge'''
#         if vision_receive[0] == 'V':
#             if vision_receive[1] == 'exit':
#                 self.vision_client_run = False
#                 logging.info("'exit' command received, start terminating program\n")
#             elif vision_receive[1] == 'al':
#                 alive_resp = self.vision.alive()
#                 print('alive(), response: {}'.format(alive_resp))
#             elif vision_receive[1] == 'cc':
#                 cc_resp = self.vision.check_cpu_speed()
#                 print('get_att(), response: {}'.format(cc_resp))
#             elif vision_receive[1] == 'gp':
#                 pose_resp = self.vision.get_pose()
#                 print('get_pose(), response: {}'.format(pose_resp))
#             elif vision_receive[1] == 'gs':
#                 status_resp = self.vision.get_status()
#                 print('get_status(), response: {}'.format(status_resp))
#             elif vision_receive[1] == 'sv':
#                 self.reset_flag = False
#                 save_resp = self.vision.save_db()
#                 time.sleep(3)
#                 print('save_db(), response: {}'.format(save_resp))
#                 self.vision_client.send_list(['V', 'next'])
#             elif vision_receive[1] == 'rs':
#                 self.reset_flag = True
#                 self.vision_thread_client_run = False
#                 time.sleep(0.2)
#                 reset_resp = self.vision.reset()
#                 print('reset(), response: {}'.format(reset_resp))
#                 time.sleep(10)
#                 self.start_background_thread()
#                 self.vision_client.send_list(['V', 'next'])
#             elif vision_receive[1] == 'bm': # Build map
#                 if vision_receive[2] is not None:
#                     start_resp = self.vision.set_start(1, [vision_receive[2]])
#                     print('set_start(), response: {}'.format(start_resp))
#                     logging.info("'Build map' command received , mapid : %s ", vision_receive[2])
#                 else:
#                     print("'Build map' command received , but no mapid")
#                     logging.info("'Build map' command received , but no mapid")
#             elif vision_receive[1] == 'um': # Use map
#                 if vision_receive[2] is not None:
#                     start_resp = self.vision.set_start(0, [vision_receive[2]])
#                     print('set_start(), response: {}'.format(start_resp))
#                     logging.info("'Use map' command received , mapid : %s ", vision_receive[2])
#                 else:
#                     print("'Use map' command received , but no mapid")
#                     logging.info("'Use map' command received , but no mapid")
#             elif vision_receive[1] == 'kbm': # Keep building map
#                 if vision_receive[2] is not None:
#                     start_resp = self.vision.set_start(2, [vision_receive[2]])
#                     print('set_start(), response: {}'.format(start_resp))
#                     logging.info("'Keep build map' command received , mapid : %s ", vision_receive[2])
#                 else:
#                     print("'Keep build map' command received , but no mapid")
#                     logging.info("'Build map' command received , but no mapid")


#         else:
#             print(str(vision_receive) + " received by vision module. Wrong potorcol ! ")


#     def end(self):
#         '''End vision program'''
#         self.vision_client_run = False
#         self.vision_client.close()
#         logging.info(" Vision module disconnect \n")

#     def end_background_thread(self):
#         '''End vision thread'''
#         self.vision_thread_client_run = False
#         self.vision_thread_client.send_list([self.vision_x, self.vision_y, self.vision_theta, self.vision_status,\
#              self.vision_client_run, self.vision_thread_client_run])
#         self.vision_thread_client.close()


# if __name__ == "__main__":
#     Vision()