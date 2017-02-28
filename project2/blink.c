#include <arduino.h>

int main(void) {
	init();
	setup();
	while (true) loop();
	return 0;
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay(100);
	digitalWrite(LED_BUILTIN, LOW);
	delay(100);
}

