#!/usr/bin/python3
import ROVER_socket
import time
import traceback
import subprocess
import sys
import threading
import logging
#import CarControl

'''Portocol'''
''' "C" to Main , "L" to LiDAR , "K" to Keyboard control , "G" to GPIO , "V" to Vision , "M" to motion '''

###                                                                   ###
###    global variables                                               ###
###                                                                   ###
commander_server = None
lidar_server = None
vision_server = None
gui_server = None
bridge_run = False
logging.basicConfig(filename='Bridge.log',filemode = 'w',level =logging.INFO)


def bridge_init():
    global commander_server , vision_server , lidar_server , gui_server , bridge_run
    try:
        commander_init()

        vision_init()
        lidar_init()
        gui_init()

        bridge_run = True
        logging.info("Bridge is ready to run ! \n")
    except:
        if commander_server != None:
            commander_server.close()
        if vision_server != None:
            vision_server.close()
        if gui_server != None:
            gui_server.close()
        bridge_run = False
        sys.exit(0)


###                                                                            ###
###          Gateway for commander communication. See ROVER_main.py            ###
###                                                                            ###
def commander_init():
    global commander_server
    try:
        logging.info("Initialize commander server\n")
        commander_server = ROVER_socket.TCP_server(50000,1)
        commander_server.send_list(['C','next'])
        logging.info("Commander connection completed !\n")

    except:
        commander_server.close()
        traceback.print_exc()
        logging.info('Bridge initializing failed at commander_init()\n')
        logging.exception("Got error : \n")
        

###                                                                            ###
###    Gateway for Vision module communication. See ROVER_vision_main.py       ###
###                                                                            ###

def vision_init():
    global vision_server,commander_server
    try:
        logging.info("Initialize vision server\n")
        vision_server = ROVER_socket.TCP_server(50001,1)
        vision_data = vision_server.recv_list()
        if vision_data == ['V','status','Alive']:
            logging.info("Vision communication successfully established !\ncommunication center get : {} \n".format(vision_data) )
            commander_server.send_list(['C','next'])
        else:
            print('Undefined communication error of Vision module, please check test message')
            logging.info("Undefined communication error of Vision module, please check test message\n")
            raise KeyboardInterrupt      
    except:
        traceback.print_exc()
        logging.info('Bridge initializing failed at vision_init()\n')
        logging.exception("Got error : \n")



###                                                                            ###
###    Gateway for RPLiDAR communication. See ROVER_vision_main.py             ###
###                                                                            ###

def lidar_init():
    global lidar_server,commander_server
    try:
        logging.info("Initialize lidar server\n")
        lidar_server = ROVER_socket.TCP_server(50002,1)
        lidar_data = lidar_server.recv_list()
        if lidar_data == ['L','status','Good']:
            logging.info("Lidar communication successfully established !\ncommunication center get : {} \n".format(lidar_data) )
            commander_server.send_list(['C','next'])
        else:
            print('Undefined communication error of Vision module, please check test message')
            logging.info("Undefined communication error of Vision module, please check test message\n")
            raise KeyboardInterrupt      
    except:
        traceback.print_exc()
        logging.info('Bridge initializing failed at lidar_init()\n')
        logging.exception("Got error : \n")



###                                                                   ###
###         Gateway for Keyboard control communication.               ###
###                                                                   ###

def gui_init() : 
    global gui_server
    try :
        gui_server = ROVER_socket.TCP_server(50003,1)
        gui_data = gui_server.recv_list()
        bridge_potorcol(gui_data)
        logging.info("GUI connection completed!")
    except :
        gui_server.close()
        traceback.print_exc()
        print('Bridge initializing fail at gui_init()')



###                                                                   ###
###    Protocol for bridge                                            ###
###                                                                   ###


'''
['S' , 'next']
    ['C' , 'next' ]

'''

def bridge_potorcol(receive_data):
    global commander_server , vision_server , bridge_run
    '''First, get commander command (ROVER_main.py)'''
    try:
        if receive_data[0] == 'C':
            if receive_data[1] == 'exit':
                lidar_server.send_list(['L','exit'])                
                vision_server.send_list(['V','exit'])
                print('All server will be close in 3 seconds')
                time.sleep(3)
                commander_server.close()
                vision_server.close()
                lidar_server.close()
                bridge_run = False
                

        elif receive_data[0] == 'K' :
            if receive_data[1] == 'Close' :
                CarControl.exit()
            elif receive_data[1] == 'w' :
                CarControl.foreward()
            elif receive_data[1] == 's' :
                CarControl.backward()
            elif receive_data[1] == 'a' :
                CarControl.turn_left()
            elif receive_data[1] == 'd' :
                CarControl.turn_right()
            elif receive_data[1] == '0' :
                CarControl.stop()
            elif receive_data[1] == '1' :
                CarControl.straight()
            elif receive_data[1] == 'fast' :
                CarControl.speed_up()
            elif receive_data[1] == 'slow' :
                CarControl.speed_down()
            elif receive_data[1] == 'next' :
                commander_server.send_list(['C','next']) 
            print(receive_data[1])
            


        elif receive_data[0] == 'V':
            if receive_data[1] == 'next':
                commander_server.send_list(['C','next'])      

        elif receive_data[0] == 'L':
            if receive_data[1] == 'next':
                commander_server.send_list(['C','next']) 

        else:
            print('{} received . Wrong potorcol  !'.format(receive_data))
            logging.info('{} received . Wrong potorcol  !'.format(receive_data))
            

    except:
        commander_server.close()
        lidar_server.close()
        vision_server.close()
        gui_server.close()
        traceback.print_exc()
        logging.exception("Got error : \n")
    




###                                                                   ###
###            Waiting for command from ROVER_main.py                 ###
###                                                                   ###
def bridge_main():
    global commander_server , vision_server , bridge_run

    while bridge_run:
        try:
            commander_data = commander_server.recv_list()
            logging.info("Bridge receive {} from commander\n".format(commander_data))
            bridge_potorcol(commander_data)

        except:
            commander_server.close()
            lidar_server.close()
            vision_server.close()
            gui_server.close()
            traceback.print_exc()
            logging.exception("Got error : \n")
            bridge_run = False

def end_bridge():
    commander_server.close()
    lidar_server.close()
    vision_server.close()
    gui_server.close()
    logging.info("Bridge close successfully")
            



if __name__ == "__main__":
    bridge_init()
    bridge_main()
    end_bridge()


# time.sleep(5)
# commander_server.close()
# stm32_server.close()












# class bridge_portocol(object):

#     def Command_potorcol(self,command):
#         if command[0] == 'C':
#             if command[1] == 'exit':
#                 stm32_server.send_list['S',command[1]]
#                 commander_server.close()
#                 stm32_server.close()

#         else:
#             print('Wrong potorcol from commander ')


