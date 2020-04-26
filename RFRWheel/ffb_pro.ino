/*
Force Feedback Wheel

Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
Copyright 2013  Saku Kekkonen
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

#include "ffb_pro.h"
#include "ffb.h"
#include "QuadEncoder.h"

#include <util/delay.h>

//------------------------------------- Defines ----------------------------------------------------------

#define AUTO_CENTER_SPRING		8//16
#define AUTO_CENTER_DAMPER		64
#define FRICTION_RAMP			128
// #define FRICTION_DIVIDER		2//64
// #define SPRING_DIVIDER			128//64

#define BOUNDARY_SPRING			400
#define BOUNDARY_FRICTION		400

const f32 gSpeedGain = 0.1f;
const f32 gSpringGain = 0.005f;

#define CALIBRATOR_INDEX_SEARCH_TORQUE	200
#define CALIBRATOR_SPD_THRESHOLD		100//8
#define CALIBRATION_DAMPER				32//250
#define CALIBRATOR_HOMING_SPEED			5
#define CALIBRATOR_HOMING_SPRING		128

#define CALIBRATION_TIMEOUT_SECONDS		10

#define SEC2FRAMES(m_sec)		(m_sec*CONTROL_FPS)
#define INDEX_INT_NUM			1

//--------------------------------------- Globals --------------------------------------------------------

f32 PROGMEM fir_coefs[NB_TAPS] =
{
	0.070573279, 
	0.208085075,
	0.311521993,
	0.275324301,
	0.133483934,
	0.015952111,
	-0.01492828,
	-1.24126E-05,
};

void cSpeedObs::Init ()
{
	mLastPos = 0;
	mLastSpeed = 0;
	mLastValueValid = false;
	mCurrentLS = 0;
	for (u8 i = 0; i < NB_TAPS; i++)
		mLastSpeeds[i] = 0;
}

f32 cSpeedObs::Update (s32 new_pos)
{
	s32 speed = new_pos - mLastPos;
	mLastPos = new_pos;
	if (mLastValueValid)
	{
		s32 t = 2;
// 		s32 avg_speed = (mLastSpeed*(16-t) + speed*t) >> 5;
// 		mLastSpeed = avg_speed;
// 		s32 avg_speed = (mLastSpeed + speed) >> 1;
// 		mLastSpeed = speed;
		mLastSpeeds[mCurrentLS] = speed;
		u8 fls = mCurrentLS;
		f32 avg_speed = 0;
		for (u8 i = 0; i < NB_TAPS; i++)
		{
			avg_speed += mLastSpeeds[fls];// *fir_coefs[i];
			if (fls == 0)
				fls = NB_TAPS - 1;
			else
				fls--;
		}
		avg_speed /= NB_TAPS;
		mCurrentLS++;
		if (mCurrentLS >= NB_TAPS)
			mCurrentLS = 0;
		return(avg_speed);
	}
	mLastValueValid = true;
	mLastSpeed = 0;
	return(0);
}

cFFB::cFFB()
{
	mAutoCenter = true;
}

//--------------------------------------- Effects --------------------------------------------------------

s32 ConstrainEffect (s32 val)
{
	return(constrain(val,-MM_MAX_MOTOR_TORQUE,MM_MAX_MOTOR_TORQUE));
}

s32 FrictionEffect (f32 spd,s32 mag)
{
	s32 cmd = constrain((abs(spd) - SPD_THRESHOLD)*FRICTION_RAMP*gSpeedGain,-mag * 7,mag * 7);
	if (spd > SPD_THRESHOLD)
		return(-cmd);
	if (spd < -SPD_THRESHOLD)
		return(cmd);
	return(0);
}

s32 DamperEffect (f32 spd,s32 mag)
{
// 	s32 abs_spd = abs(spd);
// 	if (abs_spd > SPD_THRESHOLD)
// 		return(-ConstrainEffect((spd*abs_spd*mag) / 400));
	if (spd > SPD_THRESHOLD)
		return(-ConstrainEffect(((spd - SPD_THRESHOLD)*mag)*gSpeedGain));// / FRICTION_DIVIDER));
	if (spd < -SPD_THRESHOLD)
		return(-ConstrainEffect(((spd + SPD_THRESHOLD)*mag)*gSpeedGain));// / FRICTION_DIVIDER));
	return(0);
}

s32 SpringEffect (s32 err,s32 mag)
{
	return(ConstrainEffect((err*mag)*gSpringGain));// / SPRING_DIVIDER));
}

s32 SineEffect (f32 freq,f32 t,s32 mag)
{
	return(ConstrainEffect((s32)(((f32)mag)*sin(freq * TWO_PI*t))));
}

//--------------------------------------------------------------------------------------------------------

void SetIndex ()
{
	gIndexFound = true;
// 	myEnc.set(ROTATION_MID);
}

void cCalibrator::Init (u8 state)
{
	mState = state;
	mCmd = 0;
	mSlowCnt = 0;
	mFastCnt = 0;
	mMin = 0;
	mMax = MAX_VAL;
	mEnvelopeCnt = 0;
	mIndexFoundPos = 0;
	mSpeed.Init();
	gIndexFound = false;
	GetParam(PARAM_ADDR_OFFSET,mOffset);
	if (state == CALIBRATING_INDEX)
	{
#ifdef INDEX_USE_INTERRUPTS
		attachInterrupt(INDEX_INT_NUM,SetIndex,RISING);
#endif
		cal_println("Calibrating Index");
	}
	if (state == CALIBRATING_LEFT) cal_println("Calibrating Left");
	if (state == CALIBRATION_DONE)
	{
		gIndexFound = true;
		cal_println("Calibration Done");
	}
}

void cCalibrator::SetOffset (s32 offset)
{
	mOffset = offset;
	SetParam(PARAM_ADDR_OFFSET,mOffset);
}

#define HomingCommand(m_search,m_pos)	constrain((SpringEffect(m_search- m_pos,min(CALIBRATOR_HOMING_SPRING,mEnvelopeCnt>>2))),-CALIBRATOR_INDEX_SEARCH_TORQUE,CALIBRATOR_INDEX_SEARCH_TORQUE);

s32 cCalibrator::CalcTorqueCommand (s32 raw_value)
{
	s32 spd = mSpeed.Update(raw_value);
	if (abs(spd) < CALIBRATOR_SPD_THRESHOLD)
		mSlowCnt++;
	else
		mFastCnt++;
	mEnvelopeCnt++;
	s32 search_pos = mEnvelopeCnt*min(CALIBRATOR_HOMING_SPEED,mEnvelopeCnt >> 2);
	switch (mState)
	{
	case CALIBRATING_LEFT:
		if (mCmd > -(MM_MAX_MOTOR_TORQUE / 2))
			mCmd--;
		if (mSlowCnt > SEC2FRAMES(3))
		{
			mSlowCnt = 0;
			mMin = raw_value;
			mCmd = -mCmd;
			mState = CALIBRATING_RIGHT;
			cal_println("Calibrating Right");
		}
		break;
	case CALIBRATING_RIGHT:
		if (mSlowCnt > SEC2FRAMES(3))
		{
			mCmd = 0;
			if (mFastCnt < 20)
			{
				mMin = 0;
				mState = CALIBRATION_ERROR;
				cal_println("Calibration Error");
			}
			else
			{
				mMax = raw_value;
				mState = CALIBRATING_HOMING;
				cal_print("Calibrating Done, min val = "); cal_print(mMin); cal_print(", max val = "); cal_println(mMax);
			}
		}
		break;
	case CALIBRATING_INDEX:
		mCmd = HomingCommand(search_pos,raw_value);
		if ((mSlowCnt > SEC2FRAMES(CALIBRATION_TIMEOUT_SECONDS)) || gIndexFound)
		{
			if (!gIndexFound)
			{
				mState = CALIBRATION_ERROR;
				mCmd = 0;
				cal_println("Calibration Error");
			}
			else
			{
				mState = CALIBRATING_HOMING;
				mIndexFoundPos = search_pos;
				mSpeed.Init();
			}
			mEnvelopeCnt = 0;
			mSlowCnt = 0;
#ifdef INDEX_USE_INTERRUPTS
			detachInterrupt(INDEX_INT_NUM);
#endif
		}
		break;
	case CALIBRATING_HOMING:
	{
 		if (mOffset<0)
 			search_pos = - search_pos;
		search_pos -= mOffset;// -CPR;
		mCmd = HomingCommand(search_pos,raw_value);
		if (mSlowCnt > SEC2FRAMES(CALIBRATION_TIMEOUT_SECONDS))
		{
			mState = CALIBRATION_ERROR;
			mCmd = 0;
			cal_println("Calibration Error");
		}
		else if (abs(search_pos) < CALIBRATOR_HOMING_SPEED)
		{
				mState = CALIBRATION_DONE;
			gFFB.mSpeed.Init();
		}
		break;
	}
	case CALIBRATION_ERROR:
	default:
		mCmd = 0;
		return(0);
		break;
	}
	s32 command = mCmd;
	command += DamperEffect(spd,CALIBRATION_DAMPER);
	return(command);
}

//--------------------------------------------------------------------------------------------------------

s32 cFFB::CalcTorqueCommand (s32 pos)
{
// 	if ((!Btest(pidState.status,ACTUATORS_ENABLED)) || (Btest(pidState.status,DEVICE_PAUSED)))
// 		return(0);
	s32 command = 0;
	f32 spd = mSpeed.Update(pos);
	if (gFFB.mAutoCenter)
	{
		command += SpringEffect(-pos,AUTO_CENTER_SPRING);
		command += DamperEffect(spd,AUTO_CENTER_DAMPER);
	}
	else for (u8 id = FIRST_EID; id <= MAX_EFFECTS; id++)
	{
		volatile TEffectState &ef = gEffectStates[id];
		if (Btest(ef.state,MEffectState_Allocated | MEffectState_Playing))
		{
			s32 err = ef.offset - pos;
			s32 mag = (((s32)ef.magnitude)*((s32)ef.gain))/163;
			switch (ef.type)
			{
			case USB_EFFECT_CONSTANT:
				command -= ConstrainEffect(mag);
				break;
			case USB_EFFECT_RAMP:
				break;
			case USB_EFFECT_SQUARE:
				break;
			case USB_EFFECT_SINE:
			{
				float freq = 1.0f / ((float)ef.period);
				float t = millis();
				command += SineEffect(freq,t,mag);
				break;
			}
			case USB_EFFECT_TRIANGLE:
				break;
			case USB_EFFECT_SAWTOOTHDOWN:
				break;
			case USB_EFFECT_SAWTOOTHUP:
				break;
			case USB_EFFECT_SPRING:
				command += SpringEffect(err,mag);
				break;
			case USB_EFFECT_DAMPER:
				command += DamperEffect(spd,mag);
				break;
			case USB_EFFECT_INERTIA:
				break;
			case USB_EFFECT_FRICTION:
				command += FrictionEffect(spd,mag);
				break;
			case USB_EFFECT_CUSTOM:
				break;
			default:
				break;
			}
		}
	}
	s32 bound = ROTATION_MID;
	if ((pos < -bound) || (pos > bound))
	{
		if (((spd >= 0) && (pos >= 0)) || ((spd < 0) && (pos < 0)))
			command += FrictionEffect(spd,BOUNDARY_FRICTION);
		bound += 100;
		if ((pos < -bound) || (pos > bound))
		{
			if (pos >= 0)
				pos = (bound - pos);
			else
				pos = (-bound - pos);
			command += SpringEffect(pos,BOUNDARY_SPRING);
		}
	}
	command = ConstrainEffect(command);
	return(command);
}

//--------------------------------------------------------------------------------------------------------

void FfbproSetAutoCenter(uint8_t enable)
{
	gFFB.mAutoCenter = enable;
}

void FfbproEnableInterrupts(void)
{
}

const uint8_t* FfbproGetSysExHeader(uint8_t* hdr_len)
{
	static const uint8_t header[] = {0xf0,0x00,0x01,0x0a,0x01};
	*hdr_len = sizeof(header);
	return header;
}

// effect operations ---------------------------------------------------------

static void FfbproSendEffectOper(uint8_t effectId, uint8_t operation)
{
}

void FfbproStartEffect(uint8_t effectId)
{
}

void FfbproStopEffect(uint8_t effectId)
{
}

void FfbproFreeEffect(uint8_t effectId)
{
}

// modify operations ---------------------------------------------------------

void FfbproModifyDuration(uint8_t effectId, uint16_t duration)
{
}

void FfbproSetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data,volatile TEffectState* effect)
{
	uint8_t eid = data->effectBlockIndex;
	
	/*
	USB effect data:
		uint8_t	reportId;	// =2
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t attackLevel;
		uint8_t	fadeLevel;
		uint16_t	attackTime;	// ms
		uint16_t	fadeTime;	// ms
	*/
	
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->fadeTime = data->fadeTime;
}

void FfbproSetCondition (USB_FFBReport_SetCondition_Output_Data_t* data,volatile TEffectState* effect)
{
	uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
		int8_t cpOffset;	// -128..127
		uint8_t	positiveCoefficient;	// 0..255
	*/
	effect->magnitude = max(0,(s16)data->positiveCoefficient);
	effect->offset = (((s32)data->cpOffset)*ROTATION_MID) >> 7;
}

void FfbproSetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data,volatile TEffectState* effect)
{
	uint8_t eid = data->effectBlockIndex;

	/*
	USB effect data:
		uint8_t	reportId;	// =4
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t magnitude;
		int8_t	offset;
		uint8_t	phase;	// 0..255 (=0..359, exp-2)
		uint16_t	period;	// 0..32767 ms
	*/
	
	effect->magnitude = (s16)data->magnitude;
	effect->offset = (((s32)data->offset)*ROTATION_MID)>>7;
	effect->phase = (s16)data->phase;
	effect->period = (s16)data->period;
}

void FfbproSetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data,volatile TEffectState* effect)
{
	uint8_t eid = data->effectBlockIndex;
	/*
	USB data:
		uint8_t	reportId;	// =5
		uint8_t	effectBlockIndex;	// 1..40
		int16_t magnitude;	// -255..255
	*/
	
	effect->magnitude = data->magnitude;
}

void FfbproSetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data,volatile TEffectState* effect)
{
	// FFP supports only ramp up from MIN to MAX and ramp down from MAX to MIN?
	/*
	USB effect data:
		uint8_t	reportId;	// =6
		uint8_t	effectBlockIndex;	// 1..40
		int8_t start;
		int8_t	end;
	*/
	}

int FfbproSetEffect (USB_FFBReport_SetEffect_Output_Data_t *data,volatile TEffectState* effect)
{
	uint8_t eid = data->effectBlockIndex;

	/*
	USB effect data:
		uint8_t	reportId;	// =1
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
		uint16_t	duration; // 0..32767 ms
		uint16_t	triggerRepeatInterval; // 0..32767 ms
		uint16_t	samplePeriod;	// 0..32767 ms
		uint8_t	gain;	// 0..255	 (physical 0..10000)
		uint8_t	triggerButton;	// button ID (0..8)
		uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
		uint8_t	directionX;	// angle (0=0 .. 180=0..360deg)
		uint8_t	directionY;	// angle (0=0 .. 180=0..360deg)
	*/

	effect->type = data->effectType;
	effect->gain = data->gain;
	bool is_periodic = false;

	// Fill in the effect type specific data
	switch (data->effectType)
	{
		case USB_EFFECT_SQUARE:
		case USB_EFFECT_SINE:
		case USB_EFFECT_TRIANGLE:
		case USB_EFFECT_SAWTOOTHDOWN:
		case USB_EFFECT_SAWTOOTHUP:
			is_periodic = true;
		case USB_EFFECT_CONSTANT:
		case USB_EFFECT_RAMP:
		break;
	
		case USB_EFFECT_SPRING:
		case USB_EFFECT_DAMPER:
		case USB_EFFECT_INERTIA:
		break;
		
		case USB_EFFECT_FRICTION:
		break;
		
		case USB_EFFECT_CUSTOM:	
		break;
		
		default:
		break;
	}
	
	return 1;
}

void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData,volatile TEffectState* effect)
{
	/*
	USB effect data:
	uint8_t		reportId;	// =1
	uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
	uint16_t	byteCount;	// 0..511	- only valid with Custom Force
	*/
}
