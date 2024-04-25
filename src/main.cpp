#include "main.h"
#include "lemlib/api.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "subsystems/slapper.hpp"
#include "autoSelect/selection.h"

// drive motors
pros::Motor leftFront(LEFT_FRONT, pros::E_MOTOR_GEARSET_06);
pros::Motor leftMiddle(LEFT_MIDDLE, pros::E_MOTOR_GEARSET_06);
pros::Motor leftBack(LEFT_BACK, pros::E_MOTOR_GEARSET_06);
pros::Motor rightFront(RIGHT_FRONT, pros::E_MOTOR_GEARSET_06);
pros::Motor rightMiddle(RIGHT_MIDDLE, pros::E_MOTOR_GEARSET_06);
pros::Motor rightBack(RIGHT_BACK, pros::E_MOTOR_GEARSET_06);

// motor groups
pros::MotorGroup leftMotors({leftFront, leftMiddle, leftBack});
pros::MotorGroup rightMotors({rightFront, rightMiddle, rightBack});

pros::Imu imu(IMU_PORT);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                  // left motor group
                              &rightMotors,                 // right motor group
                              11,                           // 11 inch track width
                              lemlib::Omniwheel::NEW_325,   // using new 3.25" omnis
                              450,                          // drivetrain rpm is 450
                              4                             // chase power is 2.
);

// lateral motion controller
lemlib::ControllerSettings linearController(6,      // proportional gain (kP)
                                            0,      // integral gain (kI)
                                            1,      // derivative gain (kD)
                                            3,      // anti windup
                                            1,      // small error range, in inches
                                            100,    // small error range timeout, in milliseconds
                                            4,      // large error range, in inches
                                            500,    // large error range timeout, in milliseconds
                                            0      // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2,     // proportional gain (kP)
                                             0,     // integral gain (kI)
                                             17,    // derivative gain (kD)
                                             3,     // anti windup
                                             1,     // small error range, in degrees
                                             100,   // small error range timeout, in milliseconds
                                             3,     // large error range, in degrees
                                             500,   // large error range timeout, in milliseconds
                                             0      // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
//-----------------------------end-globals----------------------------



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */



void matchLoading(float time){
    leftWing.set_state(1);
    pros::delay(300);
    leftWing.set_state(0);

    chassis.moveToPose(0, -13, 22, 2000, {.forwards=false, .maxSpeed=100});
    pros::delay(100);
    // chassis.turnTo(48, 88, 1000);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    leftWing.set_state(1);
    slapperMotor.move_velocity(60);
    pros::delay(time *1000);
    slapperMotor.move_velocity(0);
    leftWing.set_state(0);
    chassis.setBrakeMode(MOTOR_BRAKE_COAST);
}

void skillsRun() {
    //matchLoading(23);
    pros::delay(280);
    chassis.moveToPoint(23, -20, 1000, {.forwards=false});
    chassis.moveToPoint(-6, 6, 1000);
    slapperMotor.move_velocity(23);
    pros::delay(280);
    slapperMotor.move_velocity(0);
    // slapperMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    //through alley
    chassis.moveToPose(-5,36,0,5000, {.minSpeed=80});
    chassis.moveToPoint(-5,80,5000);
    chassis.waitUntilDone();
    slapperMotor.move_velocity(30);
    chassis.turnTo(0, 0, 1000);
    chassis.waitUntilDone();
    slapperMotor.move_velocity(0);
    //corner curve
    chassis.moveToPose(16,108,-90,2000, {.forwards=false});
    chassis.waitUntilDone();
    // chassis.turnTo(-16, 108, 1000);
    // chassis.waitUntilDone();
    chassis.tank(50, 50);
    pros::delay(320);
    chassis.tank(-110, -110);
    pros::delay(320);
    chassis.tank(0, 0);
  
    leftWing.set_state(0);
    chassis.moveToPose(51, 64, 128, 1000);
    chassis.moveToPose(72, 59, 90, 1000);
    chassis.turnTo(0, 0, 1000);
    chassis.waitUntilDone();
   
    leftWing.set_state(1);
    chassis.moveToPose(51, 88, 0, 1000, {.forwards=false});
    chassis.waitUntilDone();
    leftWing.set_state(0);
    chassis.moveToPoint(chassis.getPose().x, 64, 1000);
    chassis.moveToPose(88, 55, 0, 1000);
    //wings.set_state(1);
    //chassis.waitUntilDone();
    //wings.set_state(0);
   
   // chassis.turnTo(0, 85, 2000);
   // wings.set_state(1);
   // chassis.tank(-110, -110);
   // pros::delay(800);
   // wings.set_state(0);
   // chassis.tank(0, 0);
//
   // printf("x: %lf, y: %lf", chassis.getPose().x, chassis.getPose().y);

    //chassis.moveToPose(72, 36, 100, 1000,{.forwards=false});
    //wings.set_state(1);
    //chassis.waitUntilDone();
    //wings.set_state(0);
    //chassis.moveToPoint(15,110,1000, {.forwards=false});
    //chassis.moveToPoint(30,110,1000);
}
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            pros::lcd::print(0, "x: %lf", chassis.getPose().x); // x
            pros::lcd::print(1, "y: %lf", chassis.getPose().y); // y
            pros::lcd::print(2, "A: %lf", chassis.getPose().theta); // y

            //pros::lcd::print(2, "LB: %lf", leftBack.get_power()); // heading

            //pros::lcd::print(3, "RF: %f", rightFront.get_power()); // x
            //pros::lcd::print(4, "RM: %f", rightMiddle.get_power()); // y
            //pros::lcd::print(5, "RB: %f", rightBack.get_power()); // heading

            pros::delay(50);
        }
    });

    //Motor inits
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    selector::init();

}


/**
 * Runs while the robot is disabled
 */
void disabled() {
//       liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}


/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void flipout() {
    liftMotor.move_voltage(12000);
    pros::delay(326);
    liftMotor.move_voltage(0);
}

void elims() {
    flipout();
    chassis.moveToPoint(0, 42, 1000);
    chassis.turnTo(25, 42, 1000, false);
    intake.move_velocity(-600);
    pros::delay(100);
    chassis.waitUntilDone();
    leftWing.set_state(1);
    printf("====\n");
    chassis.tank(-127, -127);
    pros::delay(550);
    chassis.tank(0, 0);
    printf("========");
    leftWing.set_state(0);
    intake.move_velocity(0);
    
    chassis.tank(60, 60);
    pros::delay(280);
    chassis.tank(0, 0);

    chassis.moveToPose(-23, -9, 180, 2000, {.forwards=false});
    chassis.turnTo(42, 80, 1000);
    chassis.tank(-50, -50);
    pros::delay(280);
    chassis.tank(0, 0);
}

void tuning() {
    chassis.moveToPoint(0, 24, 6000);
    pros::delay(3000);
    chassis.turnTo(24, 24, 1000);
}


void rush_6() {         //16
    chassis.moveToPoint(0, -53, 2000, {.forwards=false});
    chassis.waitUntil(30);
   //leftWing.set_state(1);
    chassis.cancelMotion();
                            //42
    chassis.moveToPoint(20, -55, 2000, {.maxSpeed=77});
    intake.move_velocity(600);
    //chassis.waitUntil(10);      //tune
    rightWing.set_state(1);
    chassis.cancelMotion();

    chassis.moveToPose(20, -36, 99, 2000, {.maxSpeed=88});  //point
    chassis.waitUntilDone();
    //chassis.tank(0, 0);
    intake.move_velocity(0);

    chassis.moveToPoint(chassis.getPose().x + 8, chassis.getPose().y + 8, 1000, {.forwards=false, .maxSpeed=100});
    rightWing.set_state(0);

    chassis.turnTo(-10000, -42, 1000, true, 100);
    chassis.moveToPoint(-28, -40, 1000, {.maxSpeed=100});
    intake.move_velocity(-600);
    leftWing.set_state(1);
    rightWing.set_state(1);
    chassis.waitUntilDone();
    leftWing.set_state(0);
    rightWing.set_state(0);
    chassis.tank(80, 80);
    pros::delay(320);
    chassis.tank(-80, -80);
    pros::delay(320);
    chassis.tank(0, 0);
    intake.move_velocity(0);

    //chassis.moveToPoint(chassis.getPose().x + 10, chassis.getPose().y, 1000, {.forwards=false});
    chassis.moveToPoint(23, -12, 1000, {.maxSpeed=88});
    intake.move_velocity(480);
    chassis.waitUntilDone();
    pros::delay(480);
    intake.move_velocity(0);

    //chassis.moveToPoint(chassis.getPose().x - 6, chassis.getPose().y - 6, 1000, {.forwards=false});
    chassis.moveToPose(-42, -38, -120, 2000);
    pros::delay(800);
    leftWing.set_state(1);
    pros::delay(320);
    intake.move_velocity(-600);
    chassis.waitUntilDone();
    chassis.tank(-23, -23);
    intake.move_velocity(0);
    leftWing.set_state(0);
    pros::delay(1600);
    chassis.tank(0, 0);
    

    
}
    

void safe_awp() {
    intake.move_velocity(-600);
    pros::delay(800);
    intake.move_velocity(0);

    chassis.moveToPose(-8, -18, 45, 1000, {.forwards=false, .maxSpeed=48});
    chassis.waitUntilDone();
    rightWing.set_state(1);
    chassis.moveToPoint(3, -9, 1000);
    chassis.moveToPose(3, 34, 0, 2000, {.maxSpeed=80});
    intake.move_velocity(-600);
    chassis.waitUntilDone();
    pros::delay(2000);
    rightWing.set_state(0);
    leftWing.set_state(1);
    pros::delay(280);
    chassis.tank(16, 0);
    pros::delay(480);
    chassis.tank(0, 0);
     
}

void six_ball() {
    //flipout
    intake.move_velocity(-600);
    pros::delay(800);
    intake.move_velocity(0);

    intake.move_velocity(600);
    chassis.moveToPoint(0, 6, 1000);
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_velocity(0);

    chassis.moveToPoint(23, -42, 1000, {.forwards=false});
    chassis.moveToPoint(36, -45, 1000, {.forwards=false});
    chassis.moveToPoint(16, -36, 1000);

    chassis.moveToPose(14, -42, 32, 1000);
    chassis.waitUntilDone();
    leftWing.set_state(1);
    //chassis.moveToPoint(

    chassis.turnToAngle(-100, 1000);
    pros::delay(200);
    leftWing.set_state(0);
    chassis.turnToAngle(90, 1000);
    chassis.waitUntilDone();

    intake.move_velocity(-600);
    chassis.moveToPoint(36, -45, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(28, -45, 1000); //backup    

    intake.move_velocity(0);
    
    
    
}


void autonomous() {
    switch (selector::auton) {
        case 1:     //far   safe
        case -1:    //close safe
            printf("awp\n");
            safe_awp();
            break;
        case 2:     //far dis
        case -2:    //close dis
            printf("sb/r6\n");
            rush_6();
            //six_ball();
            break;
        default:
            pros::delay(10000);            
    }
}

void opcontrol() {
    //leftWing.set_state(0);
    while (true) {
        update_intake();
        update_slapper();
        update_lift();
        leftWing.driver_update_toggle();
        rightWing.driver_update_toggle();
        rachet_p.driver_update();

        // get joystick positions
        int input_brake = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

       #if 0
        if (input_brake > 100) {
            leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
            rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
            liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        } else if (input_brake < -100) {
            leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
            rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
            liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
        #endif  

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.curvature(abs(leftY) > 16 ? leftY : 0, abs(rightX) > 16 ? rightX : 0);

        pros::delay(10);
    }
}
