
#ifndef _FFB_PRO_
#define _FFB_PRO_

#include <stdint.h>
#include "ffb.h"

#define SPD_THRESHOLD		0//8
#define MM_MAX_MOTOR_TORQUE  1000//400

#define ADC_NB_BITS		10

#define VAL_NB_BITS		16
#define MAX_VAL			((1<<VAL_NB_BITS)-1)
#define MID_VAL			(1<<(VAL_NB_BITS-1))

#define MID_REPORT_X			(X_AXIS_PHYS_MAX>>1)
#define MID_REPORT_Y			(Y_AXIS_PHYS_MAX>>1)
#define MID_REPORT_Z			(Z_AXIS_PHYS_MAX>>1)

#if 0//defined(__AVR_ATmega32U4__)										// On arduino uno we only have 2 external hardware interrupt pins, on Leonardo we can use pin other interrupts
#define INDEX_USE_INTERRUPTS
#endif

#define CALIBRATING_LEFT			0x0
#define CALIBRATING_RIGHT			0x1
#define CALIBRATING_INDEX			0x2
#define CALIBRATING_HOMING			0x3
#define CALIBRATION_ERROR			0x4
#define CALIBRATION_DONE			0xff

#define NB_TAPS	9

void FfbproSetAutoCenter(uint8_t enable);

void FfbproStartEffect(uint8_t id);
void FfbproStopEffect(uint8_t id);
void FfbproFreeEffect(uint8_t id);

void FfbproModifyDuration(uint8_t effectId, uint16_t duration);

void FfbproSetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetCondition(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect);
void FfbproSetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect);
int  FfbproSetEffect(USB_FFBReport_SetEffect_Output_Data_t *data, volatile TEffectState* effect);
void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState* effect);

class cSpeedObs
{
public:
	cSpeedObs()	{ Init(); }

	void Init ();
	f32 Update(s32 new_pos);

	s32 mLastPos;
	s32 mLastSpeed;
	b8 mLastValueValid;
	s32 mLastSpeeds[NB_TAPS];
	u8 mCurrentLS;
};

class cCalibrator
{
public:
	void Init (u8 state = CALIBRATING_LEFT);
	void SetOffset (s32 offset);
	s32 CalcTorqueCommand (s32 raw_value);
	s32 GetCalibratedValue (s32 raw_value)	{ return(map(constrain(raw_value,mMin,mMax),mMin,mMax,0,MAX_VAL)); }

	s32 mCmd;
	s32 mSlowCnt;
	s32 mFastCnt;
	s32 mMin;
	s32 mMax;
	s32 mOffset;
	s32 mEnvelopeCnt;
	s32 mIndexFoundPos;
	cSpeedObs mSpeed;
	u8 mState;
};

class cFFB
{
public:
	cFFB();
	
	s32 CalcTorqueCommand (s32 pos);

	cSpeedObs mSpeed;
	b8 mAutoCenter;
};

#endif // _FFB_PRO_