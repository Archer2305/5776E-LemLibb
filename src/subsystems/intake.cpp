#include "main.h"
#include "subsystems/intake.hpp"

pros::Motor intake(INTAKE1_PORT, pros::E_MOTOR_GEAR_BLUE, false);
//pros::Motor intakeMotor2(INTAKE2_PORT, pros::E_MOTOR_GEAR_BLUE, true);

//pros::Motor_Group intake({intakeMotor1, intakeMotor2});

void update_intake() {
    int input_1 = controller.get_digital(BUTTON_INTAKE);
    int input_2 = controller.get_digital(BUTTON_OUTTAKE);

    if (input_1) {
        intake.move_velocity(600);
    } else if (input_2){
        intake.move_velocity(-600);
    } else {
        intake.move_velocity(0);
    } 
}
