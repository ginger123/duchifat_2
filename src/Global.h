#ifndef __GLOBAL__
#define __GLOBAL__

#include <hal/boolean.h>
#include "GomEPS.h"

#define STATE_MUTE 0x01
#define STATE_GS 0x02
#define GS_TIME 420

extern unsigned char states; // 0 - mute, 1 - beacon
extern gom_eps_channelstates_t glb_channels_state;

typedef struct global_param{
Boolean Mnlp_State;
unsigned char vbatt;
unsigned char cursys;
unsigned char curout3V3;
unsigned char curout5V;
unsigned char tempCOMM;
unsigned char tempEPS;
unsigned char tempBatt;
} global_param;

extern global_param glb;


void Set_Mute(Boolean bool);
void Set_Mnlp_State(Boolean state);
void Set_Vbatt(unsigned short Vbatt);
void Set_Cursys(unsigned short cursys);
void Set_Curout3V3(unsigned short curout);
void Set_Curout5V(unsigned short curout);
void Set_tempCOMM(short temp);
void Set_tempEPS(short temp);
void Set_tempBatt(short temp);
unsigned long convert_epoctime(char packet[]);
void convert_time_array(unsigned long t_l, unsigned char time[5]);
void print_array(unsigned char *arr,int length);
void switch_endian(unsigned char *in, int len);

Boolean Get_Mute();

double Min(double a, double b);
double Max(double a, double b);


#endif
