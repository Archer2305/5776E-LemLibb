#include "main.h"
#include "lemlib/api.hpp"
#include "subsystems/slapper.hpp"

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

// Inertial Sensor on port 2
pros::Imu imu(IMU_PORT);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                  // left motor group
                              &rightMotors,                 // right motor group
                              11,                           // 11 inch track width
                              lemlib::Omniwheel::NEW_325,   // using new 3.25" omnis
                              450,                          // drivetrain rpm is 450
                              6                                  
);

// lateral motion controller
lemlib::ControllerSettings linearController(8,      // proportional gain (kP)
                                            0,      // integral gain (kI)
                                            0,      // derivative gain (kD)
                                            3,      // anti windup
                                            1,      // small error range, in inches
                                            100,    // small error range timeout, in milliseconds
                                            4,      // large error range, in inches
                                            500,    // large error range timeout, in milliseconds
                                            6     // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2,     // proportional gain (8kP)
                                             0,     // integral gain (kI)
                                             16,    // derivative gain (kD)
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
    intake.move_velocity(600);
    pros::delay(480);
    chassis.moveToPoint(10, -15, 1000, {.forwards=false, .maxSpeed=100});
    chassis.waitUntilDone();
    intake.move_velocity(0);
    chassis.turnTo(42, 72, 1000);       //15 deg
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.tank(-36, -36);
    pros::delay(165);
    chassis.tank(0, 0);
    
    pros::delay(800);
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    wings.set_state(1);
    slapperMotor.move_velocity(70);
    pros::delay(time *1000);
    slapperMotor.move_velocity(0);
    wings.set_state(0);
    chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    
    pros::delay(280);
    chassis.moveToPoint(24, -24, 1000, {.forwards=false});  //push preloads in
}

void new_skills() {
    matchLoading(0);

    //along middle barrier
    chassis.moveToPoint(12, -23, 1000);
    chassis.moveToPose(17, 22, 36, 1000, {.forwards=false, .maxSpeed=88});
    chassis.waitUntilDone();
    chassis.moveToPoint(95, 22, 3000, {.forwards=false, .maxSpeed=125});
    wings.set_state(1);
    rachet_p.set_state(1);
    chassis.waitUntilDone();

    //around short barrier
    chassis.moveToPose(87, 18, -93, 1000);  //backup
    wings.set_state(0);
    chassis.moveToPoint(99, -36, 1000, {.forwards=false, .maxSpeed=92});
    chassis.moveToPoint(123, -12, 1000, {.forwards=false, .maxSpeed=92});

    //through alley 
    chassis.moveToPoint(128, 42, 1000, {.forwards=false, .maxSpeed=72});
    //rightWing.set_state(1);
    chassis.moveToPoint(125, 72, 1000, {.forwards=false, .maxSpeed=72});    //74
    chassis.waitUntilDone();    //temp

    //curve
    chassis.moveToPoint(105, 101, 1000, {.forwards=false, .maxSpeed=72});
    chassis.turnTo(0, 105, 1000, false, 88);
    chassis.waitUntilDone();

    //push #1
    chassis.tank(42, 42);
    pros::delay(300);
    chassis.tank(-127, -127);
    pros::delay(329);
    chassis.tank(42, 42);
    pros::delay(300);
    chassis.tank(-127, -127);
    pros::delay(380);
    chassis.tank(0, 0);
    rightWing.set_state(0);

    //push#2    
    chassis.moveToPoint(105, 93, 1000);    
    controller.rumble("...");
    chassis.moveToPoint(85, 60, 1000,{.forwards=false});    //64

    chassis.moveToPoint(82, 60, 1000, {.forwards=false});   //73
    chassis.moveToPoint(70, 96, 1000, {.forwards=false}); 

    chassis.moveToPose(97, 57, 90, 1000);       //back up
    chassis.moveToPoint(42, 64, 1000, {.forwards=false, .maxSpeed=100});   //go to next push

    //push#3
    chassis.moveToPoint(53, 96, 1200, {.forwards=false});
    chassis.moveToPose(80, 60, 90, 1000);       //bu
    //chassis.moveToPoint(64, 55, 1000, {.forwards=false});   //go to next push
    
    //push#4
    chassis.moveToPose(60, 101, 0, 1200, {.forwards=false});            //push fw 101
    pros::delay(320);
    wings.set_state(1);
    rightWing.set_state(1);
    
    //push # 5
    chassis.moveToPose(68, 57, 90, 1000);
    pros::delay(280);
    wings.set_state(0);
    rightWing.set_state(0);

    chassis.moveToPoint(42, 64, 1000, {.forwards=false, .maxSpeed=72});         //36
    chassis.moveToPoint(0, 96, 1000, {.forwards=false, .maxSpeed=72});  //12
    wings.set_state(1);
    chassis.moveToPoint(-8, 76, 1000, {.forwards=false, .maxSpeed=64});    //2
    //chassis.moveToPoint(16, 100, 1000, {.forwards=false, .maxSpeed=64});    //2
    chassis.turnTo(0, 0, 1000);
//wings 
    chassis.moveToPoint(12, 108, 1000, {.forwards=false});
    //chassis.moveToPoint(23, 112, 1000, {.forwards=false, .maxSpeed=80}); 

    chassis.turnTo(1000, 108, 1000, false);
    chassis.waitUntilDone();
    wings.set_state(0);

    //push#6
    chassis.tank(42, 42);
    pros::delay(300);
    chassis.tank(-127, -127);
    pros::delay(329);
    chassis.tank(42, 42);
    pros::delay(300);
    chassis.tank(-127, -127);
    pros::delay(380);
    chassis.tank(16, 16);
#if 0
#endif
    printf("x: %lf, y: %lf\n", chassis.getPose().x, chassis.getPose().y);

    //chassis.moveToPose(125, 16, 0, 2000, {.forwards=false});
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            pros::lcd::print(0, "x: %lf", chassis.getPose().x); // x
            pros::lcd::print(1, "y: %lf", chassis.getPose().y); // y
            //pros::lcd::print(2, "LB: %lf", leftBack.get_power()); // heading

            //pros::lcd::print(3, "RF: %f", rightFront.get_power()); // x
            //pros::lcd::print(4, "RM: %f", rightMiddle.get_power()); // y
            //pros::lcd::print(5, "RB: %f", rightBack.get_power()); // heading

            pros::delay(50);
        }
    });

    //Motor inits
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
    intake.move_velocity(600);
    pros::delay(320);
    intake.move_velocity(0);
}
void awp() {
    flipout();
    chassis.moveToPoint(0, 16, 1000);           //16
    chassis.moveToPose(-8, 38, -93, 1000);      //-93
    //chassis.waitUntilDone();
    pros::delay(320);
    intake.move_velocity(-600);
    pros::delay(640);
    intake.move_velocity(0);

    chassis.moveToPose(-21, 0, 42, 1600, {.forwards=false});
    chassis.turnTo(-20, -6, 1000, false);
    chassis.waitUntilDone();
    chassis.tank(-80, -80);
    pros::delay(64);
    chassis.tank(0, 0);
    wings.set_state(1);
    chassis.turnTo(22, -9, 1000, false);        //21, -8
    pros::delay(500);
    wings.set_state(0);
    chassis.turnTo(16, -10, 1000);

    chassis.turnTo(13, -9, 1600);
    chassis.waitUntilDone();
    printf("%lf, %lf\n", chassis.getPose().x, chassis.getPose().y);
    chassis.moveToPoint(4, -16, 1000); 
    chassis.waitUntilDone();
    chassis.moveToPose(25, -16, 90, 1000);     //-7
    intake.move_velocity(-600);
    pros::delay(1600);
    intake.move_velocity(0);
#if 0
#endif
}

void elims() {
    flipout();
    chassis.moveToPoint(0, 42, 1000);
    chassis.turnTo(25, 42, 1000, false);
    intake.move_velocity(-600);
    pros::delay(100);
    chassis.waitUntilDone();
    wings.set_state(1);
    printf("====\n");
    chassis.tank(-127, -127);
    pros::delay(550);
    chassis.tank(0, 0);
    printf("========");
    wings.set_state(0);
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

void six_ball() {
    flipout();

    intake.move_velocity(600);
    chassis.moveToPoint(0, 4, 1000, {.maxSpeed = 16});
    pros::delay(420);
    intake.move_velocity(0);

    chassis.moveToPoint(0, -26, 2000, {.forwards = false, .maxSpeed=88});
    chassis.moveToPose(42, -58, -84, 500, {.forwards = false, .maxSpeed=100}, true); //|
    pros::delay(170);
    //chassis.waitUntil(8);                                                             //|
    wings.set_state(1);
    //chassis.waitUntil(16);
    pros::delay(590);
    wings.set_state(0);
    chassis.waitUntilDone();

    chassis.moveToPoint(37, -55, 2000, {.forwards=false});
    chassis.tank(64, 64);
    pros::delay(100);
    chassis.tank(0, 0);

    chassis.moveToPose(32, -55, 95, 1000, {.maxSpeed = 64});
    chassis.waitUntilDone();
    intake.move_velocity(-600);
    pros::delay(250);
    chassis.tank(88, 88);
    pros::delay(380);
    chassis.tank(-80, -80);
    pros::delay(400);
    chassis.tank(0, 0);
    intake.move_velocity(0);

//get first barrier tb
    chassis.moveToPoint(51, 0, 1000, {.maxSpeed=95});
    intake.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(800);
    intake.move_velocity(0);
    chassis.moveToPoint(63, -59, 1000); //score
    pros::delay(320);
    intake.move_velocity(-600);
    chassis.waitUntilDone();
    intake.move_velocity(0);
#if 0
    //second triball
    chassis.moveToPose(55, 12, 80, 1000);
    chassis.moveToPose(60, 0, 80, 1000);
    intake.move_velocity(600);
    chassis.waitUntilDone();
    intake.move_velocity(0);

    chassis.moveToPoint(60, -59, 1000, {.forwards=false});
    wings.set_state(1);
    chassis.waitUntilDone();
    wings.set_state(0);
    chassis.moveToPoint(60, -42, 1000);

    chassis.moveToPoint(60, -59, 1000);
    pros::delay(280);
    intake.move_velocity(-600);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    
    chassis.tank(-80, -80);
    pros::delay(280);
    chassis.tank(0, 0);
#endif
}

void tuning() {
    chassis.turnTo(24, 0, 2000);
}

void autonomous() {
    new_skills();
}

void opcontrol() {
    //matchLoading(25);
    while (true) {
        update_intake();
        update_slapper();
        update_lift();
        wings.driver_update_toggle();
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

        int input_skills = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (input_skills) {
            matchLoading(22);
            //along middle barrier
            chassis.moveToPoint(12, -23, 1000);
            chassis.moveToPose(17, 24, 36, 1000, {.forwards=false, .maxSpeed=88});
            chassis.waitUntilDone();
            chassis.moveToPoint(97, 23, 3000, {.forwards=false, .maxSpeed=123});
            wings.set_state(1);
            rachet_p.set_state(1);
            chassis.waitUntilDone();
            wings.set_state(0);
        }

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // move the chassis with curvature drive
        chassis.curvature(abs(leftY) > 16 ? leftY : 0, abs(rightX) > 16 ? rightX : 0);
        // delay to save resources
        pros::delay(10);
    }
}
