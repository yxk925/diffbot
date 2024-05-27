
import time

from lx98n_driver_wiringpi import Lx98N_MotorDriverWiringpi
        
def motor_driver_test():
    """
    Test where we go forewards, backwards, left, right and stop.
    :return:
    """
    motor = Lx98N_MotorDriverWiringpi()


    motor.forward()
    print("forward")
    time.sleep(20.0)

    motor.stop()

def motor_driver_test_basic(PWM_VALUE):
    """
    Test where we go forewards, backwards.
    :return:
    """
    motor = Lx98N_MotorDriverWiringpi()

    motor.MotorSpeedSetAB(pwm_value, pwm_value)
    time.sleep(30.0)
    
    motor.stop()

    

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    pwm_value = 100
    while True:
        motor_driver_test_basic(pwm_value)
    print("Starting Motor DRiver Test...END")
