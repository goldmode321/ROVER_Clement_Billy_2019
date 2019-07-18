# # Simple demo of of the PCA9685 PWM servo/LED controller library.
# # This will move channel 0 from min to max position repeatedly.
# # Author: Tony DiCola
# # License: Public Domain
# from __future__ import division
# import time
# import Adafruit_PCA9685

# # Initialise the PCA9685 using the default address (0x40).
# pwm = Adafruit_PCA9685.PCA9685()

# pwm.set_pwm_freq(60)

# Forward_Speed = 500
# Backward_Speed = 270
# Status = 0

# import curses
 
# # get the curses screen window
# screen = curses.initscr()

# curses.noecho()
# curses.cbreak()
# screen.keypad(True)
# curses.halfdelay(5)

# #reset
# pwm.set_pwm(1, 1, 400)
# pwm.set_pwm(3, 3, 400)

# '''
# Please use arrow key to control the car
# You can press w or s to change the speed when it wa s moving
# Press w will make uit faster
# '''

# try:
#     while True:
#         char = screen.getch()
#         if char == curses.KEY_RIGHT:
#             screen.addstr(0, 0, 'Enter : Right   ')
#             pwm.set_pwm(1, 1, 300)
#         elif char == curses.KEY_LEFT:
#             screen.addstr(0, 0, 'Enter : Left    ')
#             pwm.set_pwm(1, 1, 500)
#         elif char == ord(' '):
#             screen.addstr(0, 0, 'Enter : STOP    ')
#             pwm.set_pwm(3, 3, 400)
#             Status = 0
#         elif char == curses.KEY_UP:
#             screen.addstr(0, 0, 'Enter : Forward ')
#             pwm.set_pwm(3, 3, 400)
#             pwm.set_pwm(3, 3, Forward_Speed)
#             Status = 1
#         elif char == curses.KEY_DOWN: 
#             if Status == 0:
#                 screen.addstr(0, 0, 'Enter : Backward')
#                 pwm.set_pwm(3, 3, Backward_Speed)
#                 Status = 2
#             elif Status == 1 :
#                 screen.addstr(0, 0, 'Enter : Stop    ')
#                 pwm.set_pwm(3, 3, 200)
#                 time.sleep(0.1)
#                 pwm.set_pwm(3, 3, 400)
#                 Status = 0
#         elif char == -1:
#             screen.addstr(0, 0, 'Enter :         ')
#             pwm.set_pwm(1, 1, 400)


#         elif char == ord('w') :
#             if Status == 1 and Forward_Speed < 650 :
#                 Forward_Speed = Forward_Speed + 30
#                 pwm.set_pwm(3, 3, Forward_Speed)
#                 screen.addstr(0, 0, 'FASTER          ')
#             elif Status == 2 and Backward_Speed > 190 :
#                 Backward_Speed = Backward_Speed - 30
#                 pwm.set_pwm(3, 3, Backward_Speed)
#                 screen.addstr(0, 0, 'FASTER          ') 
#         elif char == ord('s') :
#             if Status == 1 and Forward_Speed > 440 :
#                 Forward_Speed = Forward_Speed - 30                                  
#                 pwm.set_pwm(3, 3, Forward_Speed)
#                 screen.addstr(0, 0, 'SLOWER          ')
#             elif Status == 2 and Backward_Speed < 350 :
#                 Backward_Speed = Backward_Speed + 30
#                 pwm.set_pwm(3, 3, Backward_Speed)
#                 screen.addstr(0, 0, 'SLOWER          ')      

            
# except KeyboardInterrupt :
#     print('EXIT....   ')
#     pwm.set_pwm(1, 1, 400)
#     time.sleep(0.5)
#     pwm.set_pwm(3, 3, 400)
#     time.sleep(0.5)
#     curses.nocbreak(); screen.keypad(0); curses.echo()
#     curses.endwin()

    

# '''
# 可設定的速度約為412~656, 522變523時燈號會從紅色變綠色
# 167~380為後退，412~656為前進，167及656為最快
# 400可作為復歸，須先復歸才可前進變後退
# 可設定的轉向約為300~500, 300為右500為左
# '''


import keyboard
import time
while True:
    print(keyboard.is_pressed(keyboard.KEY_UP))
    print(keyboard.is_pressed('left'))
    time.sleep(0.1)