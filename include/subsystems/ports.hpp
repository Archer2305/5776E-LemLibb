#ifndef __PORTS_H__
#define __PORTS_H__

//----------drive----------
#define RIGHT_FRONT     18
#define RIGHT_MIDDLE    19
#define RIGHT_BACK      20 

#define LEFT_FRONT      -1
#define LEFT_MIDDLE     -4
#define LEFT_BACK       -3
//-------------------------

//----------intake---------
#define INTAKE1_PORT     10
#define INTAKE2_PORT     5
//-------------------------

//--------slapper---------
#define SLAPPER_PORT    15
//------------------------

//----------lift----------
#define LIFT_PORT       14
//------------------------

//---------sensors----------
#define IMU_PORT        5
//--------------------------

//---------pneumatics----------
#define LEFT_WING       'C'
#define RIGHT_WING      'A'
#define HANG_ADI        'B'
//-----------------------------

//-------------------------buttons-------------------------------
#define BUTTON_INTAKE           pros::E_CONTROLLER_DIGITAL_R1
#define BUTTON_OUTTAKE          pros::E_CONTROLLER_DIGITAL_R2

#define WINGS_ACTUATE_O         pros::E_CONTROLLER_DIGITAL_B
#define WINGS_ACTUATE_I         pros::E_CONTROLLER_DIGITAL_DOWN

#define HANG_EXT                pros::E_CONTROLLER_DIGITAL_L1
#define HANG_RET                pros::E_CONTROLLER_DIGITAL_L2

#define BUTTON_SLAPPER         pros::E_CONTROLLER_DIGITAL_A
#define BUTTON_RIGHT_WING      pros::E_CONTROLLER_DIGITAL_B
#define BUTTON_LEFT_WING       pros::E_CONTROLLER_DIGITAL_DOWN
//---------------------------------------------------------------

#endif
