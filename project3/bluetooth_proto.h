// BLUETOOTH COMMS MACROS
///////////////////////// 

// Command 0 is reserved, and represents the absence of a command.
// NB: Simple commands have their most significant bit set 
// (they are numbered 128-255). This helps us quickly check in code 
// whether to expect a data byte to follow.

// Data Commands (followed by a data byte)
#define R_VEL 1
#define R_ROT 2
#define T_PAN 3
#define T_TILT 4
// Simple commands
#define R_STOP 128
#define L_ON 129
#define L_OFF 130

