
import time

from lx98n_driver_wiringpi import Lx98N_MotorDriverWiringpi
        

def motor_driver_stop():
    """
    Test where we go forewards, backwards.
    :return:
    """
    motor = Lx98N_MotorDriverWiringpi()
    motor.stop()

    

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    motor_driver_stop()
    print("Starting Motor DRiver Test...END")

