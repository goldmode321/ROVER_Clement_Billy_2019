import rplidar
import time
import traceback
import logging
import threading
import rover_socket


class Lidar():


    def __init__(self):
        logging.basicConfig(filename='LiDAR.log', filemode='w', level=logging.INFO)
        logging.info("Initializing RPLidar")

        ###############
        self.lidar = None
        self.lidar_client = None
        self.lidar_thread_client = None
        self.lidar_run_flag = False
        self.lidar_data = []
        self.lidar_connect = False
        ###############
        try:
            logging.info("Initializing Lidar_client")
            self.lidar_client = rover_socket.TCP_client(50004)
            self.lidar_thread_client = rover_socket.UDP_client(50005)
            self.lidar_scan_port()
            if self.lidar_connect:
                self.lidar_client.send_list(['L', 'status', str(self.lidar_state[0])])
                self.lidar_run_flag = True
                self.lidar_main()
            else:
                print(("LiDAR is not initialized"))
                logging.info("LiDAR is not initialized")


        except:
            traceback.print_exc()
            logging.exception("Got error\n")
            if self.lidar_client != None:
                self.lidar_client.close()
            if self.lidar_thread_client != None:
                self.lidar_thread_client.close()


    def lidar_main(self):
        self.lidar_run_background()
        while self.lidar_run_flag:
            try:
                lidar_receive = self.lidar_client.recv_list()
                logging.info("lidar received : {} ".format(lidar_receive))
                self.lidar_protocol(lidar_receive)


            except:
                traceback.print_exc()
                logging.exception('Got error : ')
                self.lidar_run_flag = False



    def lidar_protocol(self,lidar_receive):
        try:
            if lidar_receive[0] == 'L':
                if lidar_receive[1] == 'exit':
                    self.lidar_run_flag = False
                    self.lidar_client.close()
                    self.lidar.stop()


                elif lidar_receive[1] == 'gld':
                    # logging.debug("lidar data {}".format( self.lidar_data))
                    if self.lidar_data != None:
                        self.lidar_client.send_list(['L','gld', self.lidar_data])
                    else:
                        self.lidar_client.send_list(['L','gld',"No lidar data"])


            else:
                logging.warning("Wrong portocol to Lidar communication , please check lidar_portocol or bridge protocol")
        except:
            logging.exception("lidar_protocol Got error : ")




    def lidar_run_background(self):
        thread = threading.Thread(target = self.get_lidar_data ,daemon = True)
        thread.start()


#=============================================#
#                   Liberary                  #
#=============================================#


    def lidar_scan_port(self):
        retry = 0
        self.lidar_scanning_flag = True
        self.initial_scanning_port_num = 0
        logging.info('Scanning RPLidar port')
        while self.lidar_scanning_flag:
            try:
                self.lidar_USB_port = '/dev/ttyUSB'+str(self.initial_scanning_port_num)
                logging.info('Scanning '+self.lidar_USB_port)
                self.lidar = rplidar.RPLidar(self.lidar_USB_port)
                time.sleep(0.1)
                self.lidar_state = self.get_status()
                if self.lidar_state[0] == 'Good':
                    logging.info(self.lidar_state)
                    self.lidar_scanning_flag = False
                    self.lidar_connect = True
                    logging.info("lidar initialize successuflly")
                else:
                    print(self.lidar_state)
                    print('1 or more undefined problems , plase check RPLiDAR')
                    logging.warning(str(self.lidar_state)+' ; 1 or more undefined problems , please check RPLiDAR')


            except rplidar.RPLidarException:
                # print("Not this one , system continue scanning")
                logging.info('Not this one , system continue scanning')
                self.initial_scanning_port_num += 1
                if retry < 5:
                    if self.initial_scanning_port_num > 5:
                        self.initial_scanning_port_num = 0
                        retry += 1
                        logging.warning('Rescanning RPLiDAR port')
                else:
                    self.lidar_scanning_flag = False

            except:
                traceback.print_exc()
                logging.exception("Got error\n")

    def get_status(self):
        return self.lidar.get_health()


    def stop(self):
        self.lidar.stop()  
        self.lidar.disconnect()
        logging.info("RPLidar disconnect")  

    def reconnect(self):
        self.lidar.stop()
        self.lidar = rplidar.RPLidar(self.lidar_USB_port)

    def get_lidar_object(self):
        return self.lidar

    def get_lidar_data(self):

        try:
            for scan in self.lidar.iter_scans():
                # self.lidar_data = scan
                self.lidar_data = [list(scan) for scan in scan if scan[2] > 450]
                self.lidar_thread_client.send_list([self.lidar_USB_port, self.lidar_data])
        except:
            self.reconnect()
            self.get_lidar_data()

if __name__ == "__main__":
    Lidar()
        