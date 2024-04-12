#ifndef __PORTS_H__
#define __PORTS_H__

//----------drive----------
#define RIGHT_FRONT     18
#define RIGHT_MIDDLE    19
#define RIGHT_BACK      20

#define LEFT_FRONT      -15
#define LEFT_MIDDLE     -14
#define LEFT_BACK       -13
//-------------------------

//----------intake---------
#define INTAKE1_PORT     -9
#define INTAKE2_PORT     10
//-------------------------

//--------slapper---------
#define SLAPPER_PORT    0
//------------------------

//----------lift----------
#define LIFT_PORT       0
//------------------------

//---------sensors----------
#define IMU_PORT        0
//--------------------------

//---------pneumatics----------
#define LEFT_WING       'A'
#define RIGHT_WING      'B'
#define RACHET_ADI      'C'
//-----------------------------

//-------------------------buttons-------------------------------
#define BUTTON_INTAKE           pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE          pros::E_CONTROLLER_DIGITAL_R2

// #define WINGS_ACTUATE_O         pros::E_CONTROLLER_DIGITAL_B
// #define WINGS_ACTUATE_I         pros::E_CONTROLLER_DIGITAL_DOWN

#define RACHET_ACT             pros::E_CONTROLLER_DIGITAL_L1
#define RACHET_DEACT           pros::E_CONTROLLER_DIGITAL_L2

#define BUTTON_SLAPPER         pros::E_CONTROLLER_DIGITAL_A
#define BUTTON_RIGHT_WING      pros::E_CONTROLLER_DIGITAL_B
#define BUTTONG_LEFT_WING      pros::E_CONTROLLER_DIGITAL_DOWN
//#define RACHET_ACT              
//#define RACHET_DEACT
//---------------------------------------------------------------

#endif
