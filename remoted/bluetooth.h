/*
 * bluetooth.h
 * Feb 2017
 * team 1
 */

// BLUETOOTH COMMS MACROS 
///////////////////////// 

// device IDs
#define ROOMBA 1
#define TURRET 2

// Roomba commands
#define R_FORWARD 0
#define R_BACKWARD 1
#define R_LEFT 2
#define R_RIGHT 3
#define R_FORWARD_LEFT 4
#define R_FORWARD_RIGHT 5
#define R_BACKWARD_LEFT 6
#define R_BACKWARD_RIGHT 7

// turret commands
#define T_LASER_ON 0
#define T_LASER_OFF 1
#define T_PAN_LEFT 2
#define T_PAN_RIGHT 3
#define T_TILT_UP 4
#define T_TILT_DOWN 5
// TODO: Do we need "pan and tilt" commands?

#define BT_Q_SIZE 64
