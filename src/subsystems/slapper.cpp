#include "main.h"

pros::Motor slapperMotor(SLAPPER_PORT, pros::E_MOTOR_GEAR_RED, false);

static int slapper_state = 0;
static int last_input = 0;

void update_slapper() {
    int input = controller.get_digital(BUTTON_SLAPPER);

    if (input && !last_input) {
        if (!slapper_state) {
            slapperMotor.move_velocity(60);
            slapper_state = 1;
        } else {
            slapperMotor.move_voltage(0);
            slapper_state = 0;
        }
    }

    last_input = input;
}
