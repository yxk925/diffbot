import RPi.GPIO as GPIO
import time

from lx98n_driver import Lx98N_MotorDriver
        
def motor_driver_test():
    """
    Test where we go forewards, backwards, left, right and stop.
    :return:
    """
    motor = Lx98N_MotorDriver()

    motor.stop()
    print("pre-stop")
    time.sleep(0.5)

    motor.forward()
    print("forward")
    time.sleep(2.0)

    motor.reverse()
    print("reverse")
    time.sleep(2.0)

    motor.left()
    print("left")
    time.sleep(2.0)

    motor.right()
    print("right")
    time.sleep(2.0)

    motor.stop()
    print("stop")
    time.sleep(2.0)

def motor_driver_test_basic(PWM_VALUE):
    """
    Test where we go forewards, backwards.
    :return:
    """
    motor = Lx98N_MotorDriver()

    motor.MotorSpeedSetAB(10, 10)
    time.sleep(3.0)
    motor.MotorSpeedSetAB(50, 50)
    time.sleep(3.0)
    motor.MotorSpeedSetAB(100, 100)
    time.sleep(3.0)
    
    motor.MotorSpeedSetAB(-100, 100)
    time.sleep(3.0)

    motor.MotorSpeedSetAB(100, -100)
    time.sleep(3.0)

    

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    pwm_value = 50
    while True:
        motor_driver_test_basic(pwm_value)
        pwm_value += 10
    GPIO.cleanup()
    print("Starting Motor DRiver Test...END")

