import rplidar
import numpy as np
import time
import traceback
import logging
import threading


class Lidar():

    def __init__(self, SharedVariables):
        logging.basicConfig(filename='LiDAR.log', filemode='w', level=logging.INFO)
        logging.info("Initializing RPLidar")
        self.SV = SharedVariables
        self.LI = self.SV.LI
        self.init()


    def init(self):
        try:
            logging.info("Initializing Lidar_client")
            self.lidar_scan_port()
            if self.LI.lidar_connect:
                self.LI.lidar_run = True
                self.lidar_get_data_thread = LidarGetDataThread(self.LI, self.lidar)
                # self.lidar_get_status_thread = LidarGetStatusThread(self.LI, self.lidar)

                # self.lidar_get_status_thread.start()
            else:
                print(("LiDAR is not initialized"))
                logging.info("LiDAR is not initialized")


        except:
            traceback.print_exc()
            logging.exception("Got error\n")


    def lidar_run(self):
        if self.lidar_get_data_thread.isAlive():
            print("lidar get data is running")
        else:
            self.lidar_get_data_thread = LidarGetDataThread(self.LI, self.lidar)

        # if self.lidar_get_status_thread.isAlive():
        #     print("lidar get status is running")
        # else:
        #     self.lidar_get_data_thread.start()

    def lidar_stop(self):
        self.lidar_get_data_thread.stop()
        # self.lidar_get_status_thread.stop()

#=============================================#
#                   Liberary                  #
#=============================================#


    def lidar_scan_port(self):
        retry = 0
        lidar_scanning_flag = True
        scanning_port_nu = 0
        logging.info('Scanning RPLidar port')
        while lidar_scanning_flag:
            try:
                self.LI.lidar_USB_port = '/dev/ttyUSB'+str(scanning_port_nu)
                logging.info('Scanning : {}'.format(self.LI.lidar_USB_port))
                self.lidar = rplidar.RPLidar(self.LI.lidar_USB_port)
                time.sleep(0.1)
                self.LI.lidar_state = self.lidar.get_health()
                if self.LI.lidar_state[0] == 'Good':
                    logging.info(self.LI.lidar_state)
                    lidar_scanning_flag = False
                    self.LI.lidar_connect = True
                    logging.info("lidar initialize successuflly")
                else:
                    print(self.LI.lidar_state)
                    print('1 or more undefined problems , plase check RPLiDAR')
                    logging.warning(str(self.LI.lidar_state)+' ; 1 or more undefined problems , please check RPLiDAR')


            except rplidar.RPLidarException:
                # print("Not this one , system continue scanning")
                logging.info('Not this one , system continue scanning')
                scanning_port_nu += 1
                if retry < 5:
                    if scanning_port_nu > 5:
                        scanning_port_nu = 0
                        retry += 1
                        logging.warning('Rescanning RPLiDAR port')
                else:
                    lidar_scanning_flag = False

            except:
                traceback.print_exc()
                logging.exception("Got error\n")

    def end(self):
        self.LI.lidar_run = False
        self.lidar.stop()  
        self.lidar.disconnect()
        logging.info("RPLidar disconnect")  

    def reconnect(self):
        self.lidar.stop()
        self.lidar = rplidar.RPLidar(self.LI.lidar_USB_port)


class LidarGetDataThread(threading.Thread):
    def __init__(self,SharedVariable_lidar, lidar):
        self.LI = SharedVariable_lidar
        self.lidar = lidar
        threading.Thread.__init__(self, daemon=True)
        self.start()

    def stop(self):
        self.LI.lidar_run = False

    def run(self):
        logging.info("Lidar Get Data Thread Run")
        try:
            for scan in self.lidar.iter_scans():
                self.LI.lidar_data = np.array([list(i) for i in scan if i[2] > self.LI.lidar_minimum_radius])
                self.LI.lidar_angle = np.array(np.radians(self.LI.lidar_data[:, 1] * (-1)))
                self.LI.lidar_radius = np.array(self.LI.lidar_data[:, 2])
                if not self.LI.lidar_run:
                    time.sleep(0.1)
                    raise KeyboardInterrupt

        except:
            logging.exception("Lidar Got Error")
            self.lidar.stop()
            self.lidar = rplidar.RPLidar(self.LI.lidar_USB_port)



