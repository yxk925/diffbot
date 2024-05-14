import wiringpi
import time

import rospy



class Lx98N_MotorDriverWiringpi(object):

    def __init__(self):

        #self.PIN = 18
        self.DIR_A1 = 7 # Values below are all wPis
        self.DIR_A2 = 5
        self.DIR_B1 = 9 
        self.DIR_B2 = 10
        self.PWM_A = 2
        self.PWM_B = 15

        self.PWM_RANGE = 1024
        self.PWM_DIVISOR = 50

        wiringpi.wiringPiSetup()
        
        #GPIO.setup(self.PIN, GPIO.IN, GPIO.PUD_UP)
        wiringpi.pinMode(self.DIR_A1, wiringpi.OUTPUT)
        wiringpi.pinMode(self.DIR_A2, wiringpi.OUTPUT)
        wiringpi.pinMode(self.DIR_B1, wiringpi.OUTPUT)
        wiringpi.pinMode(self.DIR_B2, wiringpi.OUTPUT)
        wiringpi.pinMode(self.PWM_A, wiringpi.PWM_OUTPUT)
        wiringpi.pinMode(self.PWM_B, wiringpi.PWM_OUTPUT)
        wiringpi.pwmSetRange(self.PWM_A, self.PWM_RANGE)
        wiringpi.pwmSetRange(self.PWM_B, self.PWM_RANGE)
        wiringpi.pwmSetClock(self.PWM_A, self.PWM_DIVISOR)
        wiringpi.pwmSetClock(self.PWM_B, self.PWM_DIVISOR)

    def set_motor(self, A1, A2, B1, B2):
        
        rospy.logdebug("set_motor, (%d, %d), (%d, %d)" %(A1, A2, B1, B2))
        
        wiringpi.digitalWrite(self.DIR_A1, A1)
        wiringpi.digitalWrite(self.DIR_A2, A2)
        wiringpi.digitalWrite(self.DIR_B1, B1)
        wiringpi.digitalWrite(self.DIR_B2, B2)

    def forward(self):
        self.set_motor(1, 0, 1, 0)

    def stop(self):
        self.set_motor(0, 0, 0, 0)


    def reverse(self):
        self.set_motor(0, 1, 0, 1)


    def left(self):
        self.set_motor(0, 1, 1, 0)


    def right(self):
        self.set_motor(1, 0, 0, 1)

    def determine_dir(self, speed):
        if (speed > 0):
            return 1,0
        elif (speed < 0):
            return 0,1
        else:
            return 0,0
        
	#Set motor speed
    def MotorSpeedSetAB(self,MotorSpeedA,MotorSpeedB):
        rospy.logdebug("MotorSpeedSetAB, %d, %d" %(MotorSpeedA, MotorSpeedB))
        if abs(MotorSpeedA) > 100 or abs(MotorSpeedB) > 100:
            rospy.logerr("[MotorSpeedSetAB]speed invalid:(%d, %d)" % (MotorSpeedA, MotorSpeedB))
            
            return
        a1,a2 = self.determine_dir(MotorSpeedA)
        b1,b2 = self.determine_dir(MotorSpeedB)

        self.set_motor(a1, a2, b1, b2)
        
        pwm_a = abs(int(self.PWM_RANGE * MotorSpeedA / 100));
        pwm_b = abs(int(self.PWM_RANGE * MotorSpeedB / 100));
        #print(" pwm_a:%d" %pwm_a)
        #print(" pwm_b:%d" %pwm_b)
        wiringpi.pwmWrite(self.PWM_A, pwm_a)
        wiringpi.pwmWrite(self.PWM_B, pwm_b)

        time.sleep(.02)

  

