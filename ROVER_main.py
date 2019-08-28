#!/usr/bin/python3
import ROVER_socket
import time
import subprocess
import traceback
import threading
import logging


process_bridge = None
process_vision = None
process_lidar = None
process_algorithm = None
commander_client = None
commander_run = False
logging.basicConfig(filename='Main.log',filemode = 'w',level =logging.INFO)
###                                                                   ###
###    Preview function                                               ###
###                                                                   ###

def help_menu():
    print(" exit : Quit software ")                 # [ 'C' , 'exit ']
    print(" kbc : Keyboard control ")
    


###                                                                   ###
###    Portocol for commander                                         ###
###                                                                   ###
'''
['C' , 'next' ]

'''
def commander_portocol(commander_receive):
    global commander_client,commander_run,process_bridge
    if commander_receive[0] == 'C':
        if commander_receive[1] == 'next':
            pass


'''###                                                                   ###
###    Keyboard control listener                                      ###
###                                                                   ###
def on_press(key):
    try:
        print(key.char)
    except AttributeError:
        if key == keyboard.Key.up :
            print('Forward')
            keyboard_client.send_list(['K', 'w'])
        elif key == keyboard.Key.down :
            print('Backward')
            keyboard_client.send_list(['K', 's'])
        elif key == keyboard.Key.right :
            print('Right')
            keyboard_client.send_list(['K', 'd'])
        elif key == keyboard.Key.left :
            print('left')
            keyboard_client.send_list(['K', 'a'])

def on_release(key):
    if key == keyboard.Key.esc:
        print('EXIT...')
        keyboard_client.send_list(['K', 'Close'])
        keyboard_client.close()
        return False
    elif key == keyboard.Key.up or key == keyboard.Key.down:
        print('Stop')
        keyboard_client.send_list(['K', '0'])
    elif key == keyboard.Key.right or key == keyboard.Key.left :
        print('Straight')
        keyboard_client.send_list(['K', '1'])
    elif key == keyboard.Key.page_up:
        print('Speed up')
        keyboard_client.send_list(['K', 'fast'])
    elif key == keyboard.Key.page_down :
        print('Speed down')
        keyboard_client.send_list(['K', 'slow'])'''


###                                                                   ###
###    Run ROVER_bridge.py (so called "Communication center (CC) ")   ###
###                                                                   ###
def commander_init():
    try:
        global commander_client, commander_run, process_bridge, process_vision, process_lidar, process_algorithm
        
        process_bridge = subprocess.Popen('python ROVER_bridge.py',shell = True)
        print('##### Initializing communication center #####')
        logging.info("Bridge - commander initialize")
        time.sleep(1)    # Wait some time for assuming Communication center(CC) work  稍微delay，以確保CC正常運作
        print("Establish TCP connection to communication center\nSend test data ['C',1,2,3]")
        commander_client = ROVER_socket.TCP_client(50000)
        commander_receive = commander_client.recv_list()
        commander_portocol(commander_receive) # Waiting for [ 'C' , 'next' ]
        logging.info("Bridge - commander initialization completed\n")


        print('\n\n##### Initializing Vision module #####')
        logging.info("Vision module initialize")
        process_vision = subprocess.Popen('python ROVER_vision_main.py',shell = True)
        commander_receive = commander_client.recv_list() # Waiting for [ 'C' , 'next' ]
        commander_portocol(commander_receive)
        logging.info("Vision module initialization completed\n")


        print('\n\n##### Initializing RPLiDAR #####')
        logging.info("RPLiDAR initialize")
        process_lidar = subprocess.Popen('python ROVER_rplidar_main.py',shell = True)
        commander_receive = commander_client.recv_list() # Waiting for [ 'C' , 'next' ]
        commander_portocol(commander_receive)
        logging.info("RPLiDAR module initialization completed\n")



        commander_run = True

            


    except:
        if process_algorithm != None:
            process_algorithm.kill()
        if process_lidar != None:
            process_lidar.kill()
        if process_vision != None:
            process_vision.kill()
        if process_bridge != None:
            process_bridge.kill()            
        commander_client.close()
        traceback.print_exc()

        logging.exception("Got error : \n")




###                                                                   ###
###    Waiting for User Command                                       ###
###                                                                   ###
def main():
    global commander_client,commander_run, process_bridge, process_vision , process_lidar , process_algorithm
    print('\n\n @@@ Program is all set, now is ready to run @@@')

    while commander_run:
        try:
            command = input('\nPlease enter command (Enter "h" for help menu) : ')
            if command == 'h':
                help_menu()

            elif command == 'exit':
                commander_client.send_list(['C','exit'])
                commander_run = False

            elif command == 'kbc':
                print('Please use arrow key to control, press Esc to exit')
                with keyboard.Listener(
                    on_press = on_press,
                    on_release = on_release,
                    suppress = True) as listener:
                    listener.join()


            else:
                commander_client.send_list(['C',command])
        
        except:
            commander_client.send_list(['C','exit'])
            commander_run = False
            
    commander_client.close()
    time.sleep(6)
    print('All program terminated')








if __name__ == "__main__":
    commander_init()
    main()
