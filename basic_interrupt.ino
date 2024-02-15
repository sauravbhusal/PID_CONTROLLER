volatile int pulseCount = 0;
volatile unsigned long lastTime = 0;

/* Good since values change a lot */
const int interruptPin = 12;
const int sampleDuration = 1000;

void setup() {
	pinMode(interruptPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(interruptPin), countPulses, FALLING); /* Instead of ISR pass the func name*/
	/* Aldo use digitalPinToInterrupt for new sketches as per the docs re*/
	Serial.begin(9600);
}

void loop() {
	unsigned long currentTime = millis();

	if (currentTime - lastTime >= sampleDuration) {
		float RPM = (pulseCount * 60000.0) / (currentTime - lastTime);
		ets_printf("RPM: %f", RPM); /* Work with UART */
		pulseCount = 0;
		lastTime = currentTime;
	}
}
void IRAM_ATTR countPulses() {
	pulseCount++;
}
