#!/usr/bin/python3
import ROVER_socket
import time
import subprocess
import traceback
import threading
import logging

process_stm32 = None
process_bridge = None
process_vision = None
process_lidar = None
process_algorithm = None
commander_client = None
commander_run = False
xbox = None
logging.basicConfig(filename='Main.log',filemode = 'w',level =logging.INFO)
###                                                                   ###
###    Preview function                                               ###
###                                                                   ###

def help_menu():
    print(" exit : Quit software ")                 # [ 'C' , 'exit ']
    print(" xt : Xbox test ")                       # None
    print(" mwx : Xbox remote control ")            # [ 'C' , 'xbox' , [x,y,z] ]
    print(" xtg : Xbox test, gpio ")
    


###                                                                   ###
###    Portocol for commander                                         ###
###                                                                   ###
'''
['C' , 'next' ]

'''
def commander_portocol(commander_receive):
    global commander_client,commander_run,xbox,process_bridge,process_stm32
    if commander_receive[0] == 'C':
        if commander_receive[1] == 'next':
            pass



###                                                                   ###
###    Initialize xbox. See TCN_xbox.py                               ###
###                                                                   ###
def xbox_init():
    global xbox
    xbox = TCN_xbox.xbox_controller()
    logging.info('Xbox start without error')


###                                                                   ###
###    Run TCN_bridge.py (so called "Communication center (CC) ")     ###
###                                                                   ###
def commander_init():
    try:
        global commander_client,commander_run, process_bridge, process_stm32 , process_vision , process_lidar , process_algorithm
        
        process_bridge = subprocess.Popen('python3 TCN_bridge.py',shell = True)
        print('##### Initializing communication center #####')
        logging.info("Bridge - commander initialize")
        time.sleep(1)    # Wait some time for assuming Communication center(CC) work  稍微delay，以確保CC正常運作
        print("Establish TCP connection to communication center\nSend test data ['C',1,2,3]")
        commander_client = TCN_socket.TCP_client(50000)
        commander_receive = commander_client.recv_list()
        commander_portocol(commander_receive) # Waiting for [ 'C' , 'next' ]
        logging.info("Bridge - commander initialization completed\n")

        print('\n\n##### Initializing Vision module #####')
        logging.info("Vision module initialize")
        process_vision = subprocess.Popen('python3 TCN_vision_main.py',shell = True)
        commander_receive = commander_client.recv_list() # Waiting for [ 'C' , 'next' ]
        commander_portocol(commander_receive)
        logging.info("Vision module initialization complete\n")


        print('\n\n##### Initializing RPLiDAR #####')
        logging.info("RPLiDAR initialize")
        process_lidar = subprocess.Popen('python3 TCN_rplidar_main.py',shell = True)
        commander_receive = commander_client.recv_list() # Waiting for [ 'C' , 'next' ]
        commander_portocol(commander_receive)
        logging.info("Vision module initialization complete\n")


        print('\n\n##### Initializing STM32 #####')
        logging.info("STM32 initialize")
        process_stm32 = subprocess.Popen('python3 TCN_STM32_main.py',shell = True)
        commander_receive = commander_client.recv_list() # Waiting for [ 'C' , 'next' ]
        commander_portocol(commander_receive)
        logging.info("STM32 initialization complete\n")



        commander_run = True

            


    except:
        if process_algorithm != None:
            process_algorithm.kill()
        if process_stm32 != None:
            process_stm32.kill()
        if process_lidar != None:
            process_lidar.kill()
        if process_vision != None:
            process_vision.kill()
        if process_bridge != None:
            process_bridge.kill()            
        commander_client.close()
        xbox.close()
        traceback.print_exc()

        logging.exception("Got error : \n")




    # p_bridge = subprocess.Popen('python TCN_STM32_main.py',shell = True)
    # print('Initializing STM32 motor controller')


###                                                                   ###
###     Waiting for User Command                                      ###
###                                                                   ###
def main():
    global commander_client,commander_run, process_bridge, process_stm32 , process_vision , process_lidar , process_algorithm
    print('\n\n @@@ Program is all set, now is ready to run @@@')

    while commander_run:
        try:
            command = input('\nPlease enter command (Enter "h" for help menu) : ')
            if command == 'h':
                help_menu()

            elif command == 'exit':
                commander_client.send_list(['C','exit'])
                commander_run = False

            elif command == 'xt':
                xbox.xbox_test()

            elif command == 'mwx':
                while not xbox.joy.Back():
                    move_command = xbox.xbox_control()
                    commander_client.send_list(['C','mwx',move_command])
                    commander_receive = commander_client.recv_list()
                    commander_portocol(commander_receive)
                commander_client.send_list(['C','stop_motor'])


            else:
                commander_client.send_list(['C',command])
        
        except:
            commander_client.send_list(['C','exit'])
            commander_run = False
            
    commander_client.close()
    xbox.close()
    time.sleep(6)
    print('All program terminated')








if __name__ == "__main__":
    xbox_init()
    commander_init()
    main()
