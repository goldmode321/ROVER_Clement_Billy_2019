# import time

# def gen():
#     a = 0
#     for i in range(1,21):
#         print(" i : {} a : {}".format(i,a))
#         a += 1
#         time.sleep(0.5)
#         yield a

# # for j in gen():
# #     print(j)
# while gen():
    
#     print(j)
import Adafruit_PCA9685
import time
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
mode = 0
run = True
num = 10
while run:
    # command = int(input("Enter0 "))
    print(num)
    pwm.set_pwm(3, mode, num)
    time.sleep(0.2)
    num += 5
    if num > 4040:
        run = False

import Adafruit_PCA9685
import time
import rover_socket
import threading

server = rover_socket.UDP_server(50008, 0, '192.168.5.2')
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

def keep_forward_after_forward():
    pwm.set_pwm(3,0,415)
    time.sleep(0.15)
    # pwm.set_pwm(3,0,400)
    # time.sleep(0.15)
def forward_after_reverse():
    pwm.set_pwm(3,0,415)
    time.sleep(0.15)
    pwm.set_pwm(3,0,400)
    time.sleep(0.15)
    pwm.set_pwm(3,0,415)
    time.sleep(0.15)
    # pwm.set_pwm(3,0,400)
    # time.sleep(0.15)
def reverse_after_forward():
    pwm.set_pwm(3,0,350)
    time.sleep(0.15)
    pwm.set_pwm(3,0,400)
    time.sleep(0.15)
    pwm.set_pwm(3,0,380)
    time.sleep(0.15)


    # pwm.set_pwm(3,0,350)
    # time.sleep(0.1)
    # pwm.set_pwm(3,0,380)
    # time.sleep(delay)
    # pwm.set_pwm(3,0,400)
    # time.sleep(delay)
    # pwm.set_pwm(3,0,380)

    # pwm.set_pwm(3,0,400)
    # time.sleep(0.15)
def keep_reversing_after_reverse():
    pwm.set_pwm(3,0,380)
    time.sleep(0.15)
    # pwm.set_pwm(3,0,400)
    # time.sleep(0.15)
def stop():
    pwm.set_pwm(3,0,400)
    time.sleep(0.15)

def get_data():
    global command
    while True:
        command = server.recv_list()
        time.sleep(0.1)

def turnning():
    global command
    while True:
        if command is not None:
            if command[0] in ['a', 'wa', 'sa']:
                pwm.set_pwm(1,0,495)
            elif command[0] in ['d', 'wd', 'sd']:
                pwm.set_pwm(1,0,315)
            else:
                pwm.set_pwm(1,0,405)
        else:
            pwm.set_pwm(1,0,405)
        time.sleep(0.1)


thread = threading.Thread(target = get_data, daemon = True)
thread.start()

thread2 = threading.Thread(target = turnning, daemon = True)
thread2.start()


car_state = 'stop'
while True:
    try:
        # command = input("Enter w/s/ : ")
        command = server.recv_list()
        print(car_state, command)
        if command is not None:
            if command[0] in ['w', 'wa', 'wd']:
                if car_state == 'stop':
                    keep_forward_after_forward()
                    car_state = 'forward'
                elif car_state == 'reverse':
                    forward_after_reverse()
                    car_state = 'forward'
                elif car_state == 'forward':
                    keep_forward_after_forward()

            elif command[0] in ['s', 'sd', 'sa']:
                if car_state == 'stop':
                    reverse_after_forward()
                    # keep_reversing_after_reverse()
                    car_state = 'reverse'
                elif car_state == 'forward':
                    reverse_after_forward()
                    car_state = 'reverse'
                elif car_state == 'reverse':
                    keep_reversing_after_reverse()
            else:
                stop()
                car_state = 'stop'
        else:
            stop()
            car_state = 'stop'



    except KeyboardInterrupt:
        server.close()
        break



delay = 0.15
pwm.set_pwm(3,0,400)
time.sleep(delay)
pwm.set_pwm(3,0,413)
time.sleep(2)

pwm.set_pwm(3,0,400)
time.sleep(0.1)
pwm.set_pwm(3,0,350)
time.sleep(0.1)
pwm.set_pwm(3,0,400)
time.sleep(0.05)
pwm.set_pwm(3,0,382)
time.sleep(3)
pwm.set_pwm(3,0,400)



delay = 0.15
pwm.set_pwm(3,0,400)
time.sleep(delay)
pwm.set_pwm(3,0,415)
time.sleep(2)
pwm.set_pwm(3,0,400)
time.sleep(0.1)


pwm.set_pwm(3,0,350)
time.sleep(0.1)
pwm.set_pwm(3,0,380)
time.sleep(delay)
pwm.set_pwm(3,0,400)
time.sleep(delay)
pwm.set_pwm(3,0,380)
time.sleep(3)
pwm.set_pwm(3,0,400)


