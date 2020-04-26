/*
Copyright 2015  Etienne Saint-Paul

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

#include "debug.h"
#include <Wire.h>
#include <digitalWriteFast.h>
#include "Config.h"
#include "QuadEncoder.h"
#include "HX711.h"
#include <USBDesc.h>

//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>

//--------------------------------------- Globals --------------------------------------------------------

s16 adc_val = 0;

u8 digital_inputs_pins[] = {3,6,7,8,11,12,A4,A3,A2,A1,A0};

u8 analog_inputs_pins[] = 
{
	ACCEL_PIN,
#ifndef USE_LOAD_CELL
	BRAKE_PIN,
#endif
#ifndef USE_QUADRATURE_ENCODER
	TURN_PIN,
#endif
};

u8 axis_shift_n_bits[] = 
{
	Z_AXIS_NB_BITS - ADC_NB_BITS,
#ifndef USE_LOAD_CELL
	Y_AXIS_NB_BITS - ADC_NB_BITS
#endif
#ifndef USE_QUADRATURE_ENCODER
	X_AXIS_NB_BITS - ADC_NB_BITS,
#endif
};

s32 analog_inputs[sizeof(analog_inputs_pins)];

u8 load_cell_channel;
s32 nb_mes;

//----------------------------------------- Options -------------------------------------------------------

#ifdef USE_DSP56ADC16S
void ss_falling()
{
	adc_val = (spi_buffer[0] << 8) + spi_buffer[1];
	spi_bp = 0;
}

ISR(SPI_STC_vect)
{
	spi_buffer[spi_bp] = SPDR;
	if (spi_bp<3)
		spi_bp++;
}
#endif

//--------------------------------------------------------------------------------------------------------

void InitADC ()
{
#ifdef USE_DSP56ADC16S
	pinMode(ADC_PIN_CLKIN,OUTPUT);
	pinMode(ADC_PIN_FSO,INPUT);
	pinMode(ADC_PIN_SDO,INPUT);
	pinMode(ADC_PIN_SCO,INPUT);

	SPCR = _BV(SPE);		// Enable SPI as slave
	SPCR |= _BV(SPIE);		// turn on SPI interrupts
	TCCR3A = 0b10100000;
	TCCR3B = 0b00010001;
	ICR3 = 8;
	OCR3A = 4;									// 1 MHz clock for the DSP56ADC16S
	attachInterrupt(0,ss_falling,FALLING);
#endif
}

b8 InitLoadCell ()
{
#ifdef USE_LOAD_CELL
	gLoadCell.power_down();
	gLoadCell.power_up();
	u32 mil = millis();
	last_send = mil;
	while ((mil - last_send) < 100)
	{
		mil = millis();
		if (gLoadCell.is_ready())
		{
			gLoadCell.set_gain(128);
			gLoadCell.tare(30);	//Assuming there is no weight on the scale at start up, reset the scale to 0
			return(true);
		}
	}
#endif
	return(false);
}

void InitInputs()
{
	for (u8 i = 0; i < sizeof(digital_inputs_pins); i++)
		pinMode(digital_inputs_pins[i],INPUT_PULLUP);

	for (u8 i = 0; i < sizeof(analog_inputs_pins); i++)
		pinMode(analog_inputs_pins[i],INPUT);

#ifdef USE_QUADRATURE_ENCODER
	myEnc.Init(ROTATION_MID + gCalibrator.mOffset);
#endif
	InitADC();
	InitLoadCell();

	nb_mes = 0;
	load_cell_channel = 0;
}

//--------------------------------------------------------------------------------------------------------

u16 ReadDigitalInputs ()
{
	u16 buttons = 0;
	u16 bmask = 1;
	for (u8 i = 0; i < sizeof(digital_inputs_pins); i++)
	{
		if (digitalRead(digital_inputs_pins[i]) == LOW)
			buttons |= bmask;
		bmask <<= 1;
	}
	return(buttons);
}

void ClearAnalogInputs ()
{
	for (u8 i = 0; i < sizeof(analog_inputs_pins); i++)
		analog_inputs[i] = 0;
	nb_mes = 0;
}

void ReadAnalogInputs ()
{
	for (u8 i = 0; i < sizeof(analog_inputs_pins); i++)
		analog_inputs[i] += analogRead(analog_inputs_pins[i]);
	nb_mes++;
}

void AverageAnalogInputs ()
{
	for (u8 i = 0; i < sizeof(analog_inputs_pins); i++)
		analog_inputs[i] = (analog_inputs[i] << axis_shift_n_bits[i]) / nb_mes;

#ifdef USE_DSP56ADC16S
	analog_inputs[TURN_INPUT] = -analog_inputs[TURN_INPUT] + 0x8000;
#endif

#ifdef USE_LOAD_CELL
	if ((gCalibrator.mState != CALIBRATING_INDEX) && gLoadCell.is_ready())
	{
		LCSYNC_LED_HIGH();
		s32 w1 = (gLoadCell.mOffset[load_cell_channel] - gLoadCell.read(/*1-*/load_cell_channel)) >> (3 + 16 - Y_AXIS_NB_BITS - load_cell_channel * 2);
		if (w1 < 10)
			w1 = 0;
		w1 = constrain(w1,0,Y_AXIS_PHYS_MAX);
// 		if (load_cell_channel == 0)
			brake = w1;
// 		else
// 			accel = w1;
// 		load_cell_channel = 1 - load_cell_channel;							// Commented : frame rate drops a lot when using 2 channels...
		LCSYNC_LED_LOW();
	}
#endif
}
