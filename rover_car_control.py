import time
import threading
import Adafruit_PCA9685
import rover_socket

class CarControl:
    def __init__(self, shared_variable_car_control):
        self.CC = shared_variable_car_control
        self.init()

    def init(self):
        try:
            self.car_control = Adafruit_PCA9685.PCA9685()
            self.car_control.set_pwm_freq(60)
        except:
            print("carControl not activated")
        self.CC.car_control_server = rover_socket.UDP_server(50011, 0, "192.168.5.2")
        self.CC.car_control_server_run = True
        Tunning(self.CC)
        ForwardOrReverse(self.CC)

    def end_car_control(self):
        self.CC.car_control_server_run = False
        time.sleep(0.1)
        self.CC.car_control_server.close()

class Tunning(threading.Thread):
    def __init__(self, shared_variable_car_control):
        self.CC = shared_variable_car_control
        threading.Thread.__init__(self, daemon=True)
        self.start()


    def run(self):
        while self.CC.car_control_server_run:

            temp = self.CC.car_control_server.recv_list()
            if temp is None and self.CC.car_control_previous_receive is not None:
                self.CC.car_control_receive = self.CC.car_control_previous_receive
                self.CC.car_control_previous_receive = None
            elif temp is None and self.CC.car_control_previous_receive is None:
                self.CC.car_control_receive = None
            else:
                self.CC.car_control_previous_receive = self.CC.car_control_receive
                self.CC.car_control_receive = temp

            if self.CC.car_control_receive is not None :
                if self.CC.car_control_receive[0] in ['a', 'wa', 'sa']:
                    self.turn_left()
                elif self.CC.car_control_receive[0] in ['d', 'wd', 'sd']:
                    self.turn_right()
                else:
                    self.straight()
            else:
                self.straight()


    def turn_left(self):
        self.CC.car_control.set_pwm(1,0,495)
        time.sleep(0.05)

    def turn_right(self):
        self.CC.car_control.set_pwm(1,0,315)
        time.sleep(0.05)

    def straight(self):
        self.CC.car_control.set_pwm(1,0,405)
        time.sleep(0.05)



class ForwardOrReverse(threading.Thread):
    def __init__(self, shared_variable_car_control):
        self.CC =shared_variable_car_control
        threading.Thread.__init__(self, daemon=True)
        self.start()

    def run(self):
        while self.CC.car_control_server_run:
            if self.CC.car_control_receive is not None:
                self.car_control_protocol(self.CC.car_control_receive[0])
            else:
                self.stop()


    def car_control_protocol(self, car_control_receive):
        if car_control_receive is not None:
            if car_control_receive in ['w', 'wa', 'wd']:
                if self.CC.car_control_state == 'stop':
                    self.forward()
                    self.CC.car_control_state = 'forward'
                elif self.CC.car_control_state == 'reverse':
                    self.forward_after_reverse()
                    self.CC.car_control_state = 'forward'
                elif self.CC.car_control_state == 'forward':
                    self.forward()

            elif car_control_receive in ['s', 'sd', 'sa']:
                if self.CC.car_control_state == 'stop':
                    self.reverse_after_forward()
                    self.CC.car_control_state = 'reverse'
                elif self.CC.car_control_state == 'forward':
                    self.reverse_after_forward()
                    self.CC.car_control_state = 'reverse'
                elif self.CC.car_control_state == 'reverse':
                    self.reverse()
            else:
                self.stop()
                self.CC.car_control_state = 'stop'


        else:
            self.stop()
            self.CC.car_control_state = 'stop'


    def forward_after_reverse(self):
        self.CC.car_control.set_pwm(3,0,self.CC.car_control_stop_pwm)
        time.sleep(0.1)
        self.CC.car_control.set_pwm(3,0,self.CC.car_control_forward_pwm + self.CC.car_control_add_speed)
        time.sleep(0.05)

    def reverse_after_forward(self):
        self.CC.car_control.set_pwm(3,0,self.CC.car_control_stop_pwm)
        time.sleep(0.1)
        self.CC.car_control.set_pwm(3,0,self.CC.car_control_backward_pwm - self.CC.car_control_add_speed)
        time.sleep(0.05)

    def stop(self):
        self.CC.car_control.set_pwm(3,0,self.CC.car_control_stop_pwm)
        time.sleep(0.05)

    def forward(self):
        self.CC.car_control.set_pwm(3, 0, self.CC.car_control_forward_pwm + self.CC.car_control_add_speed)
        time.sleep(0.05)

    def reverse(self):
        self.CC.car_control.set_pwm(3, 0, self.CC.car_control_backward_pwm - self.CC.car_control_add_speed)
        time.sleep(0.05)



