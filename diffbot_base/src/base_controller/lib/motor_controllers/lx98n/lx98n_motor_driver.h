/*
 * Author: Franz Pucher
 */

#ifndef LX98N_MOTOR_DRIVER_H
#define LX98N_MOTOR_DRIVER_H

#include <moveit/task_constructor/storage.h>

namespace diffbot {

    /** \brief Implementation of the MotorControllerIntf for the Adafruit_MotorShield
     * 
     * Implements the abstract setSpeed method from the MotorControllerIntf
     * 
     * The class makes use of the adafruit-stepper-dc-motor-featherwing library.
     * 
     * \note for more details see
     * https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/library-reference
     */
    class Lx98nMotorDriver
    {
    public:
        Lx98nMotorDriver();

        //void run(moveit::task_constructor::Direction dir);
    };

}

#endif // ADAFRUIT_MOTOR_CONTROLLER_H