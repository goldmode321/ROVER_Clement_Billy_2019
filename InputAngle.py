import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()

pwm.set_pwm_freq(60)

while True :
	a = input('Please input angle : ')
	print(a)
	if int(a) >=280 and int(a) <= 520 :
		pwm.set_pwm(1, 1, int(a))