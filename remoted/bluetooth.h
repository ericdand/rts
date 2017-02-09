/*
 * bluetooth.h
 * Feb 2017
 * team 1
 */

// BLUETOOTH COMMS MACROS 
///////////////////////// 

// Command 0 is reserved, and represents the absence of a command.
// NB: As a general rule, command which take a data argument come before those
// that do not. This helps us to quickly check whether to expect a data packet.

// device IDs
#define ROOMBA 1
#define TURRET 2

// Roomba commands
#define R_FORWARD 1
#define R_BACKWARD 2
#define R_LEFT 3
#define R_RIGHT 4
#define R_FORWARD_LEFT 5
#define R_FORWARD_RIGHT 6
#define R_BACKWARD_LEFT 7
#define R_BACKWARD_RIGHT 8

// turret commands
#define T_PAN_LEFT 1
#define T_PAN_RIGHT 2
#define T_TILT_UP 3
#define T_TILT_DOWN 4
#define T_PAN_LEFT_TILT_UP 5
#define T_PAN_LEFT_TILT_DOWN 6
#define T_PAN_RIGHT_TILT_UP 7
#define T_PAN_RIGHT_TILT_DOWN 8
#define T_LASER_ON 9
#define T_LASER_OFF 10

#define BT_Q_SIZE 64
