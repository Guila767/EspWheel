#include "Config.h"
#include "debug.h"

//--------------------------------------------------------------------------------------------------------

u8 toUpper(u8 c)
{
	if ((c >= 'a') && (c <= 'z'))
		return(c + 'A' - 'a');
	return(c);
}

void ProcessSerialPort ()
{
	if (DEBUG_SERIAL.available())
	{
		u8 c = toUpper(DEBUG_SERIAL.read());
		switch (c)
		{
#ifdef USE_QUADRATURE_ENCODER
		case 'R':gCalibrator.Init(CALIBRATING_INDEX); break;
		case 'C':
		{
			s32 o = myEnc.Read() - ROTATION_MID;
			o -= (o/CPR)*CPR;
			gCalibrator.SetOffset(o);
			DEBUG_SERIAL.print("Offset : "); DEBUG_SERIAL.println(gCalibrator.mOffset);
			break;
		}
#endif
#ifdef USE_SM_RS485
		case 'G':
			{
				SM_STATUS res = smRead1Parameter(smh,nodeAddress,SMP_ACTUAL_POSITION_FB,&turn);
				if (res != SM_OK)
					PrintResultFlags("Result",res);
				else
					DEBUG_SERIAL.println(turn);
			}
			break;
		case 'S':
			{
				s32 v = 0;
				if (DEBUG_SERIAL.available())
					v = DEBUG_SERIAL.read() - '0';
				v *= 1000;
				SM_STATUS res = smSetParameter(smh,nodeAddress,SMP_ABSOLUTE_SETPOINT,v);
				if (res != SM_OK)
					PrintResultFlags("Result",res);
				else
					DEBUG_SERIAL.println(v);
			}
			break;
		case 'I':InitSimpleMotion();break;
#endif
		}
	}
}
