
import time

from lx98n_driver_wiringpi import Lx98N_MotorDriverWiringpi
        
def motor_driver_test_basic(PWM_VALUE, sec):
    print("test PWM:%d,  %d seconds", PWM_VALUE, sec)
    motor = Lx98N_MotorDriverWiringpi()

    motor.MotorSpeedSetAB(pwm_value, pwm_value)
    time.sleep(sec)
    
    motor.stop()

    

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    pwm_value = 100
    sec = 10
    
    motor_driver_test_basic(pwm_value, 5)
    print("Starting Motor DRiver Test...END")

