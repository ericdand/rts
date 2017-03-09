#include "os.h"

void OS_Abort(unsigned int error) {

}

PID Task_Create_System(void (*f)(void), int arg) {
	return NULL;
}

PID Task_Create_RR(void (*f)(void), int arg) {
	return NULL;
}

PID Task_Create_Period(void (*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	return NULL;
}

void Task_Next(void) {

}

int Task_GetArg(void) {
	return 0;
}

CHAN Chan_Init() {
	return NULL;
}

void Send(CHAN ch, int v) {

}

int Recv(CHAN ch) {
	return 0;
}

void Write(CHAN ch, int v) {

}

unsigned int Now() {
	return 0;
}

