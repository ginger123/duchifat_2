#ifndef __GLOBAL__
#define __GLOBAL__

#include <hal/boolean.h>
#include "GomEPS.h"

#define STATE_MUTE 0x01
#define STATE_GS 0x02
#define STATE_MNLP_ON 0x04
#define STATE_MUTE_EPS 0x08
#define STATE_MNLP_ON_EPS 0x10
#define STATE_ADCS_ON_EPS 0x20
#define STATE_MNLP_ON_GROUND 0x40
#define GS_TIME 420
#define THREAD_TIMEOUT 60
#define THREAD_TIMESTAMP_LEN 5

#define MAIN_THREAD 0//0=main 1=mnlp 2=mnlplistener 3=adcs 4=reset
#define MNLP_THREAD 1
#define MNLPLISTENER_THREAD 2
#define ADCS_THREAD 3
#define RESET_THREAD 4

#define UNIX_EPOCH_TIME_DIFF 30*365*24*3600+7*24*3600

extern unsigned char states;
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


void Set_Mute(Boolean bool);// 0 is mute off ,1 is mute on 2 is mute_eps on 3 is both mutes on
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
void double_little_endian(unsigned char* d);
void kicktime(int n);
Boolean Get_Mute();

double Min(double a, double b);
double Max(double a, double b);
#endif
