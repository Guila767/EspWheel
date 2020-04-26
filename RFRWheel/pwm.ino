#include "Config.h"
#include <digitalWriteFast.h>

#ifdef USE_PWM

void InitPWM ()
{
	// Define pinMode for the pins and set the frequency for timer1.
	pinMode(DIR_PIN,OUTPUT);
	pinMode(PWM_PIN,OUTPUT);
#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
	// Timer 1 configuration : prescaler: clockI/O / 1
	// outputs enabled, phase-correct PWM, top of 1000
	// PWM frequency calculation : 16MHz / 1 (prescaler) / 2 (phase-correct) / 1000 (top) = 8 kHz
	TCCR1A = 0b10100000;
	TCCR1B = 0b00010001;
	ICR1 = MM_MAX_MOTOR_TORQUE;
#endif
}

void setPWMDir (s16 torque)		// torque between -PWM_TOP and +PWM_TOP
{
	u8 dir = 1;

 	torque = -torque;
	if (torque < 0)
	{
		torque = -torque;
		dir = 0; 
	}
	OCR1A = constrain(torque,0,MM_MAX_MOTOR_TORQUE - 10);
	digitalWriteFast(DIR_PIN,dir);
}

void setPWM (s16 torque)		// torque between -PWM_TOP and +PWM_TOP
{
	torque = (torque + MM_MAX_MOTOR_TORQUE) >> 1;
	torque = constrain(torque,10,MM_MAX_MOTOR_TORQUE - 10);
	OCR1A = torque;
}

#endif
