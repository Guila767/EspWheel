/* Force Feedback Wheel

Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)

Permission to use, copy, modify, distribute, and sell this
software and its documentation for any purpose is hereby granted
without fee, provided that the above copyright notice appear in
all copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting
documentation, and that the name of the author not be used in
advertising or publicity pertaining to distribution of the
software without specific, written prior permission.

The author disclaim all warranties with regard to this
software, including all implied warranties of merchantability
and fitness.  In no event shall the author be liable for any
special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether
in an action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance of
this software.
*/

#include "ffb.h"
#include "ffb_pro.h"
#include "DualVNH5019MotorShield.h"
#include "debug.h"
#include <Wire.h>
#include <digitalWriteFast.h>
#include <EEPROM.h>
#include "Config.h"
#include "QuadEncoder.h"
#include "HX711.h"
#include <USBDesc.h>

//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>

//--------------------------------------- Globals --------------------------------------------------------

u8 spi_buffer[4];
u8 spi_bp = 0;
u8 spi_data_ready = false;

u32 last_send;
u32 last_refresh;
b8 fault;
s32 accel,brake,turn;

cCalibrator gCalibrator;
cFFB gFFB;

extern s32 analog_inputs[];

//----------------------------------------- Options -------------------------------------------------------

#ifdef USE_LOAD_CELL
HX711 gLoadCell(PD_OUT,PD_SCK);
#endif

#ifdef USE_QUADRATURE_ENCODER
//Encoder myEnc(QUAD_ENC_PIN_A,QUAD_ENC_PIN_B,ROTATION_MID);
cQuadEncoder myEnc;
#endif

#ifdef USE_VNH5019

DualVNH5019MotorShield ms;

void stopIfFault()
{
	if (ms.getM1Fault())
	{
		DEBUG_SERIAL.println("M1 fault");
		while (1);
	}
	if (ms.getM2Fault())
	{
		DEBUG_SERIAL.println("M2 fault");
		while (1);
	}
}
#endif

//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup()
{
	last_send = last_refresh = micros();
	fault = false;
	accel = 0;
	brake = 0;
	turn = 0;

	ReadEEPROMConfig();

	DEBUG_SERIAL.begin(115200);

 	pinMode(LCSYNC_LED_PIN,OUTPUT);
 	pinMode(SYNC_LED_PIN,OUTPUT);
  	pinMode(LED_PIN, OUTPUT);

// 	pinMode(SCK,INPUT); //11,INPUT);
// 	pinMode(MISO,INPUT); //12,INPUT);
// 	pinMode(MOSI,INPUT); //13,INPUT);

#ifdef USE_QUADRATURE_ENCODER
	gCalibrator.Init(CALIBRATE_AT_INIT ? CALIBRATING_INDEX : CALIBRATION_DONE);
#else
	gCalibrator.Init(CALIBRATE_AT_INIT ? CALIBRATING_LEFT : CALIBRATION_DONE);
#endif

	InitInputs();

	FfbSetDriver(0);

#ifdef USE_VNH5019
	ms.init();
#endif

#ifdef USE_PWM
	InitPWM();
	setPWM(0);
#endif
}

//--------------------------------------------------------------------------------------------------------
void loop()
{
	s32 command = 0;
	u32 now_micros = micros();
	{
		u32 time_diff = now_micros - last_send;

#ifdef USE_QUADRATURE_ENCODER
		if ((now_micros - last_refresh) >= CONTROL_PERIOD)
		{
			SYNC_LED_HIGH();
			last_refresh = now_micros;
			turn = myEnc.Read() - ROTATION_MID - gCalibrator.mOffset;
			if (gCalibrator.mState == CALIBRATION_DONE)
				command = gFFB.CalcTorqueCommand(turn);
			else
				command = gCalibrator.CalcTorqueCommand(turn);
			setPWMDir(command);
			turn = (turn*X_AXIS_PHYS_MAX) / ROTATION_MAX;
			turn = constrain(turn,-MID_REPORT_X,MID_REPORT_X);
			ReadAnalogInputs();
			SYNC_LED_LOW();
			digitalWriteFast(LED_PIN,(abs(command) >= MM_MAX_MOTOR_TORQUE) ? HIGH : LOW);
		}
#else

		if (time_diff > (nb_mes * 125))
			ReadAnalogInputs();
#endif
		if (time_diff > SEND_PERIOD)
		{
			last_send = now_micros;

			AverageAnalogInputs();
			accel = analog_inputs[ACCEL_INPUT];
#ifndef USE_LOAD_CELL
			brake = analog_inputs[BRAKE_INPUT];
#endif
#ifndef USE_QUADRATURE_ENCODER
			turn = analog_inputs[TURN_INPUT];
			if (gCalibrator.mState == CALIBRATION_DONE)
			{
				turn = gCalibrator.GetCalibratedValue(analog_inputs[TURN_INPUT]);
				command = gFFB.CalcTorqueCommand(turn - MID_VAL);
			}
			else
				command = gCalibrator.CalcTorqueCommand(analog_inputs[TURN_INPUT]);
		}
#ifdef USE_VNH5019
			ms.setM1Speed((int)command);
#endif
#endif

			u16 buttons = ReadDigitalInputs();
			SendInputReport((s16)turn,(u16)brake,(u16)accel,buttons);
// 			DEBUG_SERIAL.print("mes :");DEBUG_SERIAL.println(raw_turn);

			ClearAnalogInputs();

			if (gCalibrator.mState != CALIBRATING_INDEX)					// Free some cpu when searching for index
				ProcessSerialPort();
		}
	}
	u16 flash_per = 2000;
	switch (gCalibrator.mState)
	{
	case CALIBRATING_HOMING:	flash_per = 200; goto flash;
	case CALIBRATING_LEFT:
	case CALIBRATING_RIGHT:
	case CALIBRATING_INDEX:		flash_per = 400; goto flash;
	case CALIBRATION_ERROR:		
flash:
		digitalWriteFast(LED_PIN,(millis() % flash_per) > (flash_per>>1) ? HIGH : LOW); 
		break;
// 	default:	analogWrite(LED_PIN,brake >> (Z_AXIS_NB_BITS - 8)); break;
	} 
}
