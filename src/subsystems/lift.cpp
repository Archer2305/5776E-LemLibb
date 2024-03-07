#include "main.h"
#include "subsystems/lift.hpp"

pros::Motor liftMotor(LIFT_PORT, pros::E_MOTOR_GEARSET_06, false);

void update_lift() {
    int input_1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    int input_2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    
    if (input_1){
        liftMotor.move_velocity(600);
    } else if (input_2){
        liftMotor.move_velocity(-600);
    } else {
        liftMotor.move_velocity(0);
    }
}

//move voltage?
