#include "Config.h"
#include <EEPROM.h>

//-------------------------------------------------------------------------------------------------------

void getParam (u8 offset,u8 *addr_to,u8 size)
{
	for (u8 i = 0; i < size; i++)
		addr_to[i] = EEPROM.read(offset + i);
}

void setParam (u8 offset,u8 *addr_to,u8 size)
{
	for (u8 i = 0; i < size; i++)
		EEPROM.write(offset + i,addr_to[i]);
}

void SetDefaultConfig ()
{
	u32 val = VERSION;
	SetParam(PARAM_ADDR_VERSION,val);
	val = 0;
	SetParam(PARAM_ADDR_OFFSET,val);
}

u32 ReadEEPROMConfig ()
{
	u32 version;
	GetParam(PARAM_ADDR_VERSION,version);
	if (version != VERSION)							// First time run, or version changed, so put default values for safety
		SetDefaultConfig();
	return(version);
}
