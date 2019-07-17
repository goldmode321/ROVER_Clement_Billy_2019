import time
import Adafruit_PCA9685
import curses

class CarControl() :
    def __init__(self) :
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.screen = curses.initscr()

        self.pwm.set_pwm_freq(60)

        self.Forward_Speed = 500
        self.Backward_Speed = 270
        self.Status = 0

        curses.noecho()
        curses.cbreak()
        self.screen.keypad(True)
        curses.halfdelay(5)

        self.pwm.set_pwm(1, 1, 400)
        self.pwm.set_pwm(3, 3, 400)

    def reset(self) :
        self.pwm.set_pwm(3, 3, 400)

    def straight(self) :
        self.screen.clear()
        if self.Status == 0 :
            self.screen.addstr(0, 0, 'Stop')
        elif self.Status == 1 :
            self.screen.addstr(0, 0, 'Forward')
        elif self.Status == 2 :
            self.screen.addstr(0, 0, 'Backward')
        self.pwm.set_pwm(1, 1, 400)

    def forward(self) :
        self.screen.addstr(0, 0, 'Forward')
        self.reset()
        self.pwm.set_pwm(3, 3, self.Forward_Speed)
        self.Status = 1

    def backward(self) :
        self.screen.addstr(0, 0, 'Backward')
        self.pwm.set_pwm(3, 3, self.Backward_Speed)
        self.Status = 2

    def turn_right(self) :
        self.screen.addstr(0, 0, 'Right')
        self.pwm.set_pwm(1, 1, 300)

    def turn_left(self) :
        self.screen.addstr(0, 0, 'Left')
        self.pwm.set_pwm(1, 1, 500)

    def stop(self) :
        self.screen.clear()
        self.screen.addstr(0, 0, 'STOP')
        self.pwm.set_pwm(3, 3, 400)
        self.Status = 0

    def speed_up(self, speed) :
        if self.Status == 1 and self.Forward_Speed < 650 :
            self.Forward_Speed += speed
            self.pwm.set_pwm(3, 3, self.Forward_Speed)
            self.screen.addstr(0, 0, 'FASTER')
        elif self.Status == 2 and self.Backward_Speed > 190 :
            self.Backward_Speed -= speed
            self.pwm.set_pwm(3, 3, self.Backward_Speed)
            self.screen.addstr(0, 0, 'FASTER') 

    def speed_down(self, speed) :
        if self.Status == 1 and self.Forward_Speed > 440 :
            self.Forward_Speed -= speed
            self.pwm.set_pwm(3, 3, self.Forward_Speed)
            self.screen.addstr(0, 0, 'SLOWER')
        elif self.Status == 2 and self.Backward_Speed < 350 :
            self.Backward_Speed += speed
            self.pwm.set_pwm(3, 3, self.Backward_Speed)
            self.screen.addstr(0, 0, 'SLOWER')   

    def exit(self) :
        self.screen.clear()
        self.screen.addstr(0, 0, 'EXIT...')
        self.pwm.set_pwm(1, 1, 400)
        time.sleep(0.5)
        self.pwm.set_pwm(3, 3, 400)
        time.sleep(0.5)
        curses.nocbreak(); self.screen.keypad(0); curses.echo()
        curses.endwin()

def main() :
    a = CarControl()
    screen = curses.initscr()
    screen.keypad(True)
    curses.noecho()
    curses.cbreak()
    curses.halfdelay(5)

    try :
        while True :
            char = screen.getch()
            screen.clear()
            if char == curses.KEY_RIGHT :
                a.turn_right()
            elif char == curses.KEY_LEFT :
                a.turn_left()
            elif char == curses.KEY_UP :
                a.forward()
            elif char == curses.KEY_DOWN :
                a.backward()
            elif char == ord(' ') :
                a.stop()
            elif char == ord('w') :
                a.speed_up(30)
            elif char == ord('s') :
                a.speed_down(30)
            elif char == -1 :
                a.straight()
                a.stop()
    except KeyboardInterrupt :
        a.exit()

if __name__ == '__main__':
    main()

'''
可設定的速度約為412~656, 522變523時燈號會從紅色變綠色
167~380為後退，412~656為前進，167及656為最快
400可作為復歸，須先復歸才可前進變後退
可設定的轉向約為300~500, 300為右500為左
'''