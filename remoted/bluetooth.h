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
// Commands that are followed by a data byte
#define R_VEL 1
#define R_ROT (1 << 1)
// Simple commands
#define R_STOP (1 << 5)

// turret commands
// Data commands
#define T_PAN 1
#define T_TILT (1 << 1)
// Simple commands
#define T_LASER_ON (1 << 5)
#define T_LASER_OFF (1 << 6)

#define BT_Q_SIZE 64
