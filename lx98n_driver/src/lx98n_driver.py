import RPi.GPIO as GPIO
import time



class Lx98N_MotorDriver(object):

    def __init__(self):

        self.PIN = 18
        self.PWMA1 = 20 # 6
        self.PWMA2 = 21 # 13
        self.PWMB1 = 6 # 20
        self.PWMB2 = 13 #21
        self.D1 = 12
        self.D2 = 26

        self.PWM = 50

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.PWMA1, GPIO.OUT)
        GPIO.setup(self.PWMA2, GPIO.OUT)
        GPIO.setup(self.PWMB1, GPIO.OUT)
        GPIO.setup(self.PWMB2, GPIO.OUT)
        GPIO.setup(self.D1, GPIO.OUT)
        GPIO.setup(self.D2, GPIO.OUT)
        self.p1 = GPIO.PWM(self.D1, 500)
        self.p2 = GPIO.PWM(self.D2, 500)
        self.p1.start(50)
        self.p2.start(50)

    def set_motor(self, A1, A2, B1, B2):
        GPIO.output(self.PWMA1, A1)
        GPIO.output(self.PWMA2, A2)
        GPIO.output(self.PWMB1, B1)
        GPIO.output(self.PWMB2, B2)

    def forward(self):
        self.set_motor(1, 0, 1, 0)

    def stop(self):
        self.set_motor(0, 0, 0, 0)


    def reverse(self):
        self.set_motor(0, 1, 0, 1)


    def left(self):
        self.set_motor(1, 0, 0, 0)


    def right(self):
        self.set_motor(0, 0, 1, 0)

    def determine_dir(self, speed):
        if (speed > 0):
            return 1,0
        elif (speed < 0):
            return 0,1
        else:
            return 0,0
        
	#Set motor speed
    def MotorSpeedSetAB(self,MotorSpeedA,MotorSpeedB):
        a1,a2 = self.determine_dir(MotorSpeedA)
        b1,b2 = self.determine_dir(MotorSpeedB)

        self.set_motor(a1, a2, b1, b2)
        
        self.p1.ChangeDutyCycle(abs(MotorSpeedA))
        self.p2.ChangeDutyCycle(abs(MotorSpeedB))

        time.sleep(.02)

  

