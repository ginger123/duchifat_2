/*
- * mmnlp.h
- *
- *  Created on: Jan 8, 2016
- *      Author: itay
- */


#ifndef MNLP_H_
#define MNLP_H_

//#include "main.h"

#define sqnce_length		256
#define TIME_TABLE_START	12
#define MNLP_TT_EOT			0x55
#define sqnc_eot			0xFE
#define OBC_EOT				0xfe
#define MNLP_ON_CMD			0xF1
#define MNLP_LDP_CMD		0x05
#define MNLP_LDP_LENGTH		142
#define MNLP_OFF_CMD		0xF2
#define MNLP_DATA_SIZE 		174


#define MNLP_SCRIPT_ADDRESS 0x10000
#define MNLP_SCRIPT_MAX_SIZE 100
#define HEADER_LENGTH 10
#define NUM_SCRIPTS 7
#define MAX_SQNC_NUM 25
#define TO_SEND_LENGTH 3

#define GPIO_04	PIN_GPIO04
#define GPIO_05	PIN_GPIO05
#define GPIO_06	PIN_GPIO06
#define GPIO_07 PIN_GPIO07
//#define SIMULATION 1

/*
  * first element in the arrays saves the array's length
  * times_table  arry of the sequences to activate and when in a 24 hours format
  * sq[sqnc num][sqnc data]  the sequnces of the script in two TwoDimensional array,
  * int len  script length
  * int start_time  UTC format time to activate the script
  * char header[10]  the scripts's header
  * */
typedef struct sequence_com {
	int deltaTime;
	char CMD_ID;
	char LEN;
	char SEQ_CNT;
	unsigned char to_send[5];
	unsigned char *LDP_params;
}sequence_com;

typedef struct script_parse
{
	int t_t_len; // LEngth THingY
	int t_t[2][100]; // time_table
	int unix_start_time; // UTC
	struct sequence_com mnlp_com[5][MAX_SQNC_NUM];
	int sqnc_len[5];
	char LDP[MNLP_LDP_LENGTH];
}script_parse;

int set_time_table(int current_idx);
void set_definition_sequence(int active_idx,int sqnc_start);
void get_new_script(struct script_parse *helper);



void taskmnlp();//task for the mnlp operation

#ifdef SIMULATION
void _sqncTask(char * args);//task for the sqnc
#else
void _sqncTask(void * args);//task for the sqnc
#endif
void _mnlplistener(void* pvParameters);// readi9ng from the mnlp
void mnlp_listener();

unsigned short Fletcher16(unsigned char* data, int count);//check sum for the uploaded script


/*
  * Receives a script struct , a counter and a sequence and sends a command to the mNLP
  * return 0 if everything is OK.
  * return 1 if received as a response from the mNLP an error packet. */
int send_mnlp_cmd(unsigned char * sqnc_ptr);//send command to the mnlp

int obc_su_on();//turn on the

int obc_su_off();//turn on the

void turn_on_payload();

void turn_off_payload();

void error_protocol();

int check_new_script();//return 1 if ther's a new script to start

void update_script();//updates the running script

void check_timesTable();//check is ther's a new sequence to run

unsigned long get_script_start_time(int script_idx);// get the epoch start time from a script :)

void parse_script(short active_idx);

extern int scripts_adresses[NUM_SCRIPTS];

#endif /* MNLP_H_ */
