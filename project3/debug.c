#ifndef NDEBUG
#include <avr/io.h>
#include <stdio.h>
char initialized;
char buf[128];

#define BAUD 19200
#include <util/setbaud.h>
void debug_init() {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
	UCSR0B = _BV(TXEN0);
	initialized = 1;
}

void write_trace(char *M, char *file, int line) {
	if (!initialized) debug_init();

	sprintf(buf, "%s:%d %s\n", file, line, M);
	char *c = buf;
	while(*c != '\0') {
		while(!(UCSR0A & _BV(UDRE0)));
		UDR0 = *c++;
	}
}
#endif
