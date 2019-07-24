import ROVER_rplidar
import time
import traceback
import logging
import ROVER_socket

lidar = None 
lidar_client = None
lidar_run_flag = False
logging.basicConfig(filename='LiDAR_main.log',filemode = 'w',level =logging.INFO)


def init():
    global lidar_client, lidar , lidar_run_flag 
    try:
        logging.info("Initializing Lidar_client")
        lidar_client = ROVER_socket.TCP_client(50002)
        logging.info("Initializing RPLidar")
        lidar = ROVER_rplidar.Lidar()
        lidar_status = lidar.get_status()
        logging.info("lidar initialize successuflly")
        lidar_client.send_list(['L','status',str(lidar_status[0])])
        lidar_run_flag = True

    except:
        traceback.print_exc()
        logging.exception("Got error\n")
        lidar_client.close()

def main():
    global lidar_run_flag
    while lidar_run_flag:
        try:
            lidar_receive = lidar_client.recv_list()
            logging.info("lidar received : {} ".format(lidar_receive))
            lidar_portocol(lidar_receive)

        except:
            traceback.print_exc()
            logging.exception('Got error : ')
            lidar_run_flag = False

def end():
    lidar_client.close()
    lidar.stop()
    logging.info('Lidar closed successfully')


def lidar_portocol(lidar_receive):
    global lidar_run_flag
    if lidar_receive[0] == 'L':
        if lidar_receive[1] == 'exit':
            lidar_run_flag = False

    else:
        logging.warning("Wrong portocol to Lidar communication , please check lidar_portocol or bridge portocol")





if __name__ == "__main__":
    init()
    main()
    end()    

