#include <avr/io.h>

#define BAUD 19200
#include <util/setbaud.h>
void usart_init(void) {
    UBRR1H = UBRRH_VALUE;
    UBRR1L = UBRRL_VALUE;
#if USE_2X
    UCSR1A |= _BV(U2X1);
#else
    UCSR1A &= ~_BV(U2X1);
#endif
	// Tx and Rx, with Rx completion interrupts enabled.
    UCSR1B = _BV(TXEN1) | _BV(RXEN1) | _BV(RXCIE1);
}

void usart_tx(uint8_t b) {
    while(!(UCSR1A & _BV(UDRE1)));
    UDR1 = b;
}

