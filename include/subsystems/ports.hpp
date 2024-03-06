#ifndef __PORTS_H__
#define __PORTS_H__

//----------drive----------
#define RIGHT_FRONT     10
#define RIGHT_MIDDLE     4
#define RIGHT_BACK       3

#define LEFT_FRONT      -8
#define LEFT_MIDDLE     -6
#define LEFT_BACK       -5
//-------------------------

//----------intake---------
#define INTAKE1_PORT     1
#define INTAKE2_PORT     13
//-------------------------

//--------slapper---------
#define SLAPPER_PORT    15
//------------------------

//----------lift----------
#define LIFT_PORT       14
//------------------------

//---------sensors----------
#define IMU_PORT        0
//--------------------------

//---------pneumatics----------
#define LEFT_WING       'C'
#define RIGHT_WING      'D'
#define RACHET_ADI      'A'
//-----------------------------

//-------------------------buttons-------------------------------
#define BUTTON_INTAKE           pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE          pros::E_CONTROLLER_DIGITAL_R2

#define WINGS_ACTUATE_O         pros::E_CONTROLLER_DIGITAL_B
#define WINGS_ACTUATE_I         pros::E_CONTROLLER_DIGITAL_DOWN

#define RACHET_ACT             pros::E_CONTROLLER_DIGITAL_Y
#define RACHET_DEACT           pros::E_CONTROLLER_DIGITAL_RIGHT

#define BUTTON_SLAPPER         pros::E_CONTROLLER_DIGITAL_A
#define BUTTON_RIGHT_WING      pros::E_CONTROLLER_DIGITAL_B
#define BUTTONG_LEFT_WING      pros::E_CONTROLLER_DIGITAL_DOWN
//#define RACHET_ACT              
//#define RACHET_DEACT
//---------------------------------------------------------------

#endif
