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
// NB: The numbering of these commands matters!
// We make assumptions about it in code. Don't change these lightly.
#define R_FORWARD 1
#define R_BACKWARD 2
#define R_LEFT 3
#define R_RIGHT 4
#define R_FORWARD_LEFT 5
#define R_FORWARD_RIGHT 6
#define R_BACKWARD_LEFT 7
#define R_BACKWARD_RIGHT 8

// turret commands
#define T_LASER_ON 0
#define T_LASER_OFF 1
#define T_PAN_LEFT 2
#define T_PAN_RIGHT 3
#define T_TILT_UP 4
#define T_TILT_DOWN 5
// TODO: Do we need "pan and tilt" commands?

#define BT_Q_SIZE 64
