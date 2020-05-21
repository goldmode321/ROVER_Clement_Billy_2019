
import time
import traceback
import logging
import threading
import multiprocessing
from multiprocessing import Queue
import numpy as np
import rplidar

class Lidar:
    '''
    This is the object that provide the control API for RPLiDAR on ROVER
    '''
    def __init__(self, SharedVariables):
        '''
        Go to tcn_shared_variable to view/change the list of variables
        in tcn_shared_variable.Lidar 
        '''
        logging.basicConfig(filename='LiDAR.log', filemode='w', level=logging.INFO)
        logging.info("Initializing RPLidar")
        self.SV = SharedVariables
        self.LI = self.SV.LI
        self.COM = self.SV.COM
        self.COM.command_lidar['li stop'] = self.stop
        self.COM.command_lidar["li start"] = self.start
        self.COM.command_lidar["gld"] = lambda: print(self.LI.lidar_angle)
        self.COM.command_lidar["exit l"] = self.end
        self.COM.command_lidar["li pid"] = lambda: print(self.LI.lidar_process_pid)
        self.COM.command_lidar["li mm"] = self.change_lidar_detect_range
        self.init()


    def init(self):
        try:
            logging.info("Initializing Lidar_client")
            self.lidar_scan_port()
            if self.LI.lidar_connect:
                self.start()
            else:
                print(("LiDAR is not initialized"))
                logging.info("LiDAR is not initialized")


        except:
            traceback.print_exc()
            logging.exception("Got error\n")


    def start(self):
        if not hasattr(self, "lidar_get_data_thread"):
            self.lidar_get_data_thread = LidarGetDataThread(self.SV, self.lidar)
        else:
            if self.lidar_get_data_thread.isAlive():
                print("lidar get data is running")
            else:
                self.lidar_get_data_thread = LidarGetDataThread(self.SV, self.lidar)

    def stop(self):
        self.lidar_get_data_thread.stop()
        print("Stop lidar thread and process")

    def lidar_scan_port(self):
        retry = 0
        lidar_scanning_flag = True
        self.LI.lidar_port_num = 0
        logging.info('Scanning RPLidar port')
        while lidar_scanning_flag:
            try:
                self.LI.lidar_usb_port = '/dev/ttyUSB'+str(self.LI.lidar_port_num)
                logging.info('Scanning : {}'.format(self.LI.lidar_usb_port))
                self.lidar = rplidar.RPLidar(self.LI.lidar_usb_port)
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
                    logging.warning(
                        str(self.LI.lidar_state)+' ; 1 or more undefined problems ,\
                            please check RPLiDAR')


            except rplidar.RPLidarException:
                # print("Not this one , system continue scanning")
                logging.info('Not this one , system continue scanning')
                self.LI.lidar_port_num += 1
                if retry < 5:
                    if self.LI.lidar_port_num > 5:
                        self.LI.lidar_port_num = 0
                        retry += 1
                        logging.warning('Rescanning RPLiDAR port')
                else:
                    lidar_scanning_flag = False

            except:
                traceback.print_exc()
                logging.exception("Got error\n")

    def end(self):
        self.lidar_get_data_thread.stop()
        if self.lidar_get_data_thread.isAlive():
            self.lidar_get_data_thread.join()
        self.lidar.stop()
        self.lidar.disconnect()
        logging.info("RPLidar disconnect")

    def reconnect(self):
        self.lidar.stop()
        self.lidar = rplidar.RPLidar(self.LI.lidar_usb_port)

    def change_lidar_detect_range(self):
        maximum_radius = int(input("Lidar max radius"))
        minimum_radius = int(input("Lidar min radius"))
        if maximum_radius > minimum_radius:
            self.LI.lidar_maximum_radius = maximum_radius
            self.LI.lidar_minimum_radius = minimum_radius
        else:
            print("Lidar max radius must be larger than min radius")

class LidarGetDataThread(threading.Thread):
    def __init__(self, SharedVariables, lidar):
        self.SV = SharedVariables
        self.LI = self.SV.LI
        self.VI = self.SV.VI
        self.LOBS = self.SV.LOBS
        self.lidar = lidar
        threading.Thread.__init__(self, daemon=True)
        self.input_queue = Queue()
        self.output_queue = Queue()
        self.command_queue = Queue()
        self.lidar_process = LidarGetDataProcess(
            self.LI,
            self.lidar,
            self.input_queue,
            self.output_queue,
            self.command_queue,
        )
        self.LI.lidar_process_pid = self.lidar_process.pid
        self.end_process = False
        print("LI Process pid : {}".format(self.LI.lidar_process_pid))
        self.start()

    def stop(self):
        self.end_process = True
        self.command_queue.put("exit")
        try:
            self.lidar_process.join(1)
            self.LI.lidar_run = False
        except:
            print("Lidar process can not be stopped. Please check")

    def run(self):
        self.end_process = False
        self.LI.lidar_run = True
        i = 0
        while self.LI.lidar_run:
            if i%5 == 0:
                if self.input_queue.empty():
                    self.input_queue.put(self.LI)
                else:
                    try:
                        self.input_queue.get(False)
                        self.input_queue.put(self.LI)
                    except:
                        traceback.print_exc()

            try:
                temp_li = self.output_queue.get(timeout=0.1)
                self.update_li(temp_li)
                self.update_lobs()

            except Exception:
                pass

            # Restart Lidar if it fail
            if not self.lidar_process.is_alive() and not self.end_process:
                self.lidar_process = LidarGetDataProcess(
                    self.LI, self.lidar,
                    self.input_queue, self.output_queue, self.command_queue,
                )
                self.LI.lidar_process_pid = self.lidar_process.pid
                print("LI Process got problem, reinitialize")
                print("LI Process pid : {}".format(self.LI.lidar_process_pid))

            i += 1

    def update_li(self, temp_li):
        self.LI.lidar_angle = temp_li.lidar_angle
        self.LI.lidar_radius = temp_li.lidar_radius
        self.LI.lidar_obs_x = temp_li.lidar_obs_x
        self.LI.lidar_obs_y = temp_li.lidar_obs_y

    def update_lobs(self):
        self.LOBS.local_obstacle_x = np.round(np.cos(self.LI.lidar_angle - \
            self.VI.vision_angle_radian)* \
            self.LI.lidar_radius + self.VI.vision_x, 0)
        self.LOBS.local_obstacle_y = np.round(np.sin(self.LI.lidar_angle - \
            self.VI.vision_angle_radian)*\
            self.LI.lidar_radius + self.VI.vision_y, 0)

class LidarGetDataProcess(multiprocessing.Process):
    def __init__(
            self,
            SharedVariable_LI,
            lidar,
            input_queue,
            output_queue,
            command_queue,
        ):
        self.LI = SharedVariable_LI
        self.lidar = lidar
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.command_queue = command_queue
        super().__init__(daemon=True)

        self.start()

    def run(self):
        logging.info("Lidar Get Data Thread Run")
        self.LI.lidar_run = True
        try:
            for scan in self.lidar.iter_scans():
                try:
                    if not self.input_queue.empty():
                        temp_li = self.input_queue.get(False)
                        self.LI.lidar_minimum_radius = temp_li.lidar_minimum_radius
                        self.LI.lidar_maximum_radius = temp_li.lidar_maximum_radius
                    if not self.command_queue.empty():
                        command = self.command_queue.get(False)
                        if command == 'exit':
                            print("Lidar Process 'exit' command received, terminate process")
                            raise KeyboardInterrupt
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    traceback.print_exc()

                self.LI.lidar_data = np.array([
                    list(i) for i in scan if i[2] > self.LI.lidar_minimum_radius\
                        and i[2] < self.LI.lidar_maximum_radius and i[0] > 14
                ])
                self.LI.lidar_angle = np.array(np.radians(self.LI.lidar_data[:, 1]*(-1) + 90))
                self.LI.lidar_radius = np.array(self.LI.lidar_data[:, 2]) / 10
                self.LI.lidar_obs_x = np.round(np.cos(self.LI.lidar_angle)*self.LI.lidar_radius, 0)
                self.LI.lidar_obs_y = np.round(np.sin(self.LI.lidar_angle)*self.LI.lidar_radius, 0)

                if self.output_queue.empty():
                    self.output_queue.put(self.LI)
                else:
                    try:
                        self.output_queue.get(False)
                        self.output_queue.put(self.LI)
                    except:
                        traceback.print_exc()

                if not self.LI.lidar_run:
                    time.sleep(0.1)
                    raise KeyboardInterrupt
        except IndexError:
            traceback.print_exc()
            print("LIDAR DEBUG : Index error")
        except KeyboardInterrupt:
            print("Lidar stop")
            self.lidar.stop()
        except:
            traceback.print_exc()
            self.lidar.stop()
            self.lidar = rplidar.RPLidar(self.LI.lidar_usb_port)
            self.LI.lidar_run = False
