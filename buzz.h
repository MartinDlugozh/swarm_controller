#ifndef BUZZ_H_
#define BUZZ_H_

#include "Arduino.h"

#define BUZZER_PIN 				7
#define BUZZER_OFF				0
#define BUZZER_ON 				1

volatile uint32_t 		buzzer_timer 		= 0;
uint16_t 				buzzer_delay 		= 0;
uint8_t					buzzer_state 		= 0;

void buzz_init(void)
{
	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);
}

void buzz(uint16_t duration_ms)
{
	digitalWrite(BUZZER_PIN, HIGH);
	buzzer_state = BUZZER_ON;
	buzzer_delay = duration_ms;
	buzzer_timer = millis();
}

void buzz_update(void)
{
	if((buzzer_state == BUZZER_ON) && ((millis() - buzzer_timer) > buzzer_delay))
	{
		digitalWrite(BUZZER_PIN, LOW);
		buzzer_state = BUZZER_OFF;
	}
}

#endif /* BUZZ_H_ */
