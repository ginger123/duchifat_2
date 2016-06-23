#include "Global.h"

global_param glb;
unsigned char states;
gom_eps_channelstates_t glb_channels_state;



double Min(double a, double b)
{
	if(b>a) return a;
	return b;
}

double Max(double a, double b)
{
	if(a>b) return a;
	return b;
}



void Set_Mute(Boolean bool)
{
	if(bool)
	{
		states = states | STATE_MUTE;
	}
	else
	{
		states = states & !(STATE_MUTE);
	}
}

void Set_Mnlp_State(Boolean state)
{
	glb.Mnlp_State = state;
}

void Set_Vbatt(unsigned short Vbatt)//in mV
{
	glb.vbatt = (unsigned char)Max(0,Min(255,((double)20*Vbatt)/1000-60));
}

void Set_Cursys(unsigned short cursys)//in mA
{
	glb.cursys = (unsigned char)Max(0,Min(255,((double)127*cursys)/1000+127));
}

void Set_Curout3V3(unsigned short curout)//in mA
{
	glb.curout3V3 = (unsigned char)Max(0,Min(255,(double)40*curout));
}

void Set_Curout5V(unsigned short curout)//in mA
{
	glb.curout5V = (unsigned char)Max(0,Min(255,(double)40*curout/1000));
}

void Set_tempCOMM(short temp)
{
	glb.tempCOMM = (unsigned char)Max(0,Min(255,(double)4*temp+60));
}

void Set_tempEPS(short temp)
{
	glb.tempEPS = (unsigned char)Max(0,Min(255,(double)4*temp+60));
}

void Set_tempBatt(short temp)
{
	glb.tempBatt = (unsigned char)Max(0,Min(255,(double)4*temp+60));
}


Boolean Get_Mute()
{
	if(states & STATE_MUTE)
	{
		return TRUE;
	}
	return FALSE;
}
