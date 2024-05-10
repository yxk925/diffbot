
import time

from lx98n_driver_wiringpi import Lx98N_MotorDriverWiringpi
        
def test_dir():
    """
    Test where we go forewards, backwards, left, right and stop.
    :return:
    """
    motor = Lx98N_MotorDriverWiringpi()

    motor.stop()
    print("pre-stop")
    time.sleep(0.1)

    motor.MotorSpeedSetAB(100, 100)
    
    motor.forward()
    print("forward")
    time.sleep(3.0)

    motor.reverse()
    print("reverse")
    time.sleep(3.0)

    motor.left()
    print("left")
    time.sleep(3.0)

    motor.right()
    print("right")
    time.sleep(3.0)

    motor.stop()
    print("stop")
    time.sleep(2.0)

def test_speed():
    """
    Test where we go forewards, backwards.
    :return:
    """
    print("==test speed==")
    print("init")
    motor = Lx98N_MotorDriverWiringpi()

    print("set speed 10,10")
    motor.MotorSpeedSetAB(10, 10)
    time.sleep(3.0)
    print("set speed 30,30")
    motor.MotorSpeedSetAB(30, 30)
    time.sleep(3.0)
    print("set speed 50,50")
    motor.MotorSpeedSetAB(50, 50)
    time.sleep(3.0)
    print("set speed 75,75")
    motor.MotorSpeedSetAB(75, 75)
    time.sleep(3.0)
    print("set speed 100,100")
    motor.MotorSpeedSetAB(100, 100)
    time.sleep(3.0)
    
    print("set speed -100,100")
    motor.MotorSpeedSetAB(-100, 100)
    time.sleep(3.0)

    print("set speed 100,-100")
    motor.MotorSpeedSetAB(100, -100)
    time.sleep(3.0)

    motor.stop()
    print("stop")
    time.sleep(2.0)
    

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    test_dir()
    test_speed()
    print("Starting Motor DRiver Test...END")

