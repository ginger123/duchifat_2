/*
- * mNLP.c
- *
- *  Created on: Jan 8, 2016
- *      Author: itay
- */

#include "mnlp.h"
#include "main.h"
#include <time.h>
#include <stdio.h>
#ifdef SIMULATION
#include "util.h"
#endif

int uartstart, sqnc, active_sqnc, deactive_sqnc, tt_ctr;
script_parse current_script;
int sqnc_on, sqnc_complete;
short sqnc_locations[6];
int scripts_adresses[]={0x12000,0x14000,0x16000,0x18000,0x1A000,0x1C000,0x1E000};
unsigned char mnlp_err;
int turnover=0;
unsigned long timeout_mnlp;


//-------------------------------------------------------------------------------------
//----------------mnlp-task------------------------------------------------------------
//-------------------------------------------------------------------------------------



void taskmnlp()//task for the mnlp operation
{
#ifndef SIMULATION
	xTaskHandle task_mNLP_sqnc;
	unsigned long cur_t;

#endif
	int active_idx=-1,new_active_idx;
	int day_time;

	int date;
	char args[2];
	unsigned char * script_ptr=NULL;
	tt_ctr = 0;
	int i;

	// demi allocation
#ifndef SIMULATION
	script_ptr = (unsigned char *)calloc(10,sizeof(char));
#endif

	// ONLY FOR VKI TEST
	if (0)
	{
	turn_on_payload();
	for (i=0;i<600;i++)
	{
		kicktime(MNLP_THREAD);
		vTaskDelay(1000);
	}
	turn_off_payload();
	}

	while (1)
	{
		kicktime(MNLP_THREAD);
		// check the need to activate and parse a new script
		//printf("check for new script\n");
		new_active_idx = check_new_script();		

		//script_ptr = &script_array[active_idx][0];
		if (new_active_idx != active_idx && new_active_idx !=-1)
			//for (active_script = 0; active_script < 7;active_script++)
		{
			active_idx = new_active_idx;
			// parse file - set time table and sequences
			printf("new active script: %d\n", new_active_idx);
			parse_script(active_idx);
			print_array((unsigned char *)sqnc_locations,12);
			//restart time table counter
			tt_ctr = 0;

#ifdef SIMULATION						
			script_ptr= &script_array[active_idx][0];
#else
			free(script_ptr);
			script_ptr = (unsigned char *)calloc(sqnc_locations[0],sizeof(char));
			FRAM_read(script_ptr,scripts_adresses[active_idx],sqnc_locations[0]);
#endif

		}

		if (active_idx>=0)
		{

			// check if need to activate sequence
#ifdef SIMULATION			
			day_time = get_day_time();						
#else
			Time t;
			Time_get(&t);

			day_time = t.hours *3600 + t.minutes*60 +t.seconds;
#endif
			if (( day_time >= 0) && ( day_time <120) && (turnover==1))
			{
				printf("its a new day!!!\n");
				turnover =0;
			}

			// Check if time for running sequence			 
			date = script_ptr[TIME_TABLE_START+tt_ctr*4] +script_ptr[TIME_TABLE_START+tt_ctr*4 + 1]*60 + script_ptr[TIME_TABLE_START+tt_ctr*4 + 2]*3600;
			if (day_time>date)
			{

				if (day_time<date+30)
				{
					// activate script
					sqnc_on = 1;
					args[0] = active_idx;
					args[1] = (unsigned char) script_ptr[TIME_TABLE_START+tt_ctr*4+3];

					if (args[1]!=MNLP_TT_EOT)
					{
#ifdef SIMULATION
						sqnc_on = 1;
						_sqncTask(args);
#else
						if ( (states & STATE_MNLP_ON_EPS) && (states & STATE_MNLP_ON_GROUND))
						{
							sqnc_on = 1;
							xTaskGenericCreate(_sqncTask, (const signed char*)"taskSqnce", 1024, (void *)args, configMAX_PRIORITIES - 2, &task_mNLP_sqnc, NULL, NULL);
						}
#endif
						tt_ctr += 1;
					}
					else
					{
						printf("reached end of time\n");
						tt_ctr = 0;
						turnover = 1;
					}
				}
				else 
				{
					if (turnover == 0)
					{
						args[1] = (unsigned char) script_ptr[TIME_TABLE_START+tt_ctr*4+3];
						if (args[1]==MNLP_TT_EOT)
						{
							printf("reached end of time\n");
							tt_ctr = 0;
							turnover = 1;
						}
						else
						{
							printf("missed sequence!! - add time table counter %d\n",tt_ctr+1);
							tt_ctr += 1;
						}
					}
					//
				}

				//error handling
				if(states & STATE_MNLP_ON)
				{
					Time_getUnixEpoch(&cur_t);
					if(cur_t-timeout_mnlp>= MNLP_TIMEOUT)
					{
						printf("timeout\n");
						timeout_mnlp=cur_t;
						build_save_error_packets(active_idx,0);//got a timeout error
						error_handle();
						if(sqnc_on == 1)
						{
								vTaskDelete(task_mNLP_sqnc);
								sqnc_on = 0;
						}
					}
					if(mnlp_err!=0)
					{
						build_save_error_packets(active_idx,mnlp_err);//got an actual error from mnlp
						error_handle();
						mnlp_err=0;
						if(sqnc_on == 1)
						{
								vTaskDelete(task_mNLP_sqnc);
								sqnc_on = 0;
						}
					}

				}
			}
		}

#ifndef SIMULATION
		// delete task
		if(sqnc_on == 1 && sqnc_complete == 1)
		{
			vTaskDelete(task_mNLP_sqnc);
			sqnc_on = 0;
			sqnc_complete = 0;
		}
#endif
		vTaskDelay(850);
	}
}

void _sqncTask(void *args)//task for the sqnc
{
	unsigned char * arg_ptr;

	int sqnc_num,active_idx,sqnc_len;
	unsigned char * sqnc_ptr;
	int delay;
	arg_ptr = (unsigned char *)args;

	sqnc_num = (int) arg_ptr[1] - 0x41;
	active_idx = (int) arg_ptr[0];

	printf("i'm in a sqnce :)\n");
	// load sequence from memory
	sqnc_len = sqnc_locations[sqnc_num + 1] - sqnc_locations[sqnc_num];

#ifdef SIMULATION
	sqnc_ptr = &script_array[active_idx][sqnc_locations[sqnc_num]];
#else    
	sqnc_ptr = (unsigned char *)calloc(sqnc_len, sizeof(char));
	FRAM_read(sqnc_ptr, scripts_adresses[active_idx]+ sqnc_locations[sqnc_num], sqnc_len);
#endif

	//activate sequence
	while (sqnc_ptr[2]!=OBC_EOT)
	{
		delay = sqnc_ptr[0] + sqnc_ptr[1] * 60;
		vTaskDelay(delay*1000);

		//execute command
		send_mnlp_cmd(sqnc_ptr + 2);

		sqnc_ptr += (sqnc_ptr[3]+4);
	}

	sqnc_complete = 1;
#ifndef SIMULATION
	while (1)
	{
		vTaskDelay(5000);
	}
#endif

}

unsigned short Fletcher16(unsigned char* data, int count) {
	unsigned short sum1 = 0;
	unsigned short sum2 = 0;
	int index;
	for (index = 0; index < count; ++index) {
		sum1 = (sum1 + data[index]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}
	return (sum2 << 8) | sum1;
}



int send_mnlp_cmd(unsigned char * sqnc_ptr)//send command to the mnlp
{
	char CMD_ID = sqnc_ptr[0];
	UARTbus bus = bus0_uart;

	switch (CMD_ID)
	{
	case ((char)0xf1):
				{
		printf("switch on MNLP\n");
#ifndef SIMULATION
		turn_on_payload();
		Time_getUnixEpoch(&timeout_mnlp);
#endif
		break;
				}
	case ((char)0xf2) :
				{
		printf("switch off MNLP\n");
#ifndef SIMULATION
		turn_off_payload();
#endif
		break;
				}
	default:
	{
		printf("Send Command \n");
#ifdef SIMULATION
		UART_write(sqnc_ptr, sqnc_ptr[1]+2);
#else
		UART_write(bus,sqnc_ptr, sqnc_ptr[1]+2);
		print_array(sqnc_ptr,sqnc_ptr[1]+2);
#endif
		break;
	}
	}

	return 0;
}




int check_new_script()
{
	int i;
	unsigned long cur_time;
	unsigned long script_start_time = 0;
	unsigned long latest_current_script = 0;
	unsigned char temp_t[4];
	int current_script_idx = -1;

	// read current epoch time
#ifdef SIMULATION
	cur_time = (unsigned long) time(NULL) +3 * 3600;
#else
	// Get unix time
	Time_getUnixEpoch(&cur_time);
#endif

	//go over all scripts saved in memory and read start times
	for (i = 0; i < NUM_SCRIPTS; i++)
	{
		script_start_time = 0;
		// read current time from script
#ifdef SIMULATION
		script_start_time = get_script_start_time(i);
		//cur_day_time = 1624965405+999;
#else
		// read from sequence
		FRAM_read(temp_t, scripts_adresses[i]+2, 4);
		int k = 0;
		int j = 1;
		for(;k<4;k++)
		{
			script_start_time += temp_t[k]*j;
			j*=256;
		}
		script_start_time = script_start_time + UNIX_EPOCH_TIME_DIFF;
		//printf("current day time :%lu  mnlp  script start time: %lu script number: %d\n\n",cur_time, script_start_time,i );

#endif
		//compare to current time		

		if (script_start_time > UNIX_EPOCH_TIME_DIFF && cur_time > script_start_time && script_start_time >latest_current_script)
		{
			//printf("found: current day time :%lu  mnlp  script start time: %lu  script number: %d\n\n",cur_time, script_start_time,i );
			current_script_idx = i;
			latest_current_script = script_start_time;
		}

	}

	return current_script_idx;
}


void parse_script(short active_idx)
{
	unsigned char* script_ptr,*script_ptr_first;
	int sqnc_ctr;
	int byte_ctr;
	short length;
	// read script from memory
#ifdef SIMULATION
	script_ptr = &script_array[active_idx][0];
	length = script_ptr[1]*256 + script_ptr[0];
#else
	FRAM_read((unsigned char *)&length, scripts_adresses[active_idx], 2);
	script_ptr_first = (unsigned char *)calloc(length,sizeof(char));
	script_ptr = script_ptr_first;
	FRAM_read(script_ptr, scripts_adresses[active_idx], length);
	// read from memory
#endif

	// search for each sequence
	script_ptr = script_ptr+TIME_TABLE_START;
	sqnc_ctr = 0;
	byte_ctr = TIME_TABLE_START;
	int cmd_len;
	length=length-2;
	// go over time table
	script_ptr+=3;
	byte_ctr +=3;
	while (*script_ptr!=MNLP_TT_EOT)
	{
		script_ptr+=4;
		byte_ctr+=4;
	}
	sqnc_locations[0]=byte_ctr+1;
	sqnc_ctr = 1;
	script_ptr++;
	byte_ctr++;

	// go over sequences
	while (byte_ctr<length && sqnc_ctr<5)
	{
		if (script_ptr[2] == OBC_EOT)
		{
			byte_ctr=byte_ctr+5;
			script_ptr+=5;
			sqnc_locations[sqnc_ctr]=byte_ctr;
			sqnc_ctr++;
		}
		else
		{
			cmd_len = script_ptr[3];
			byte_ctr=byte_ctr+cmd_len+4;
			script_ptr =script_ptr+cmd_len+4;
		}

	}
	printf("found %d sequences\n",sqnc_ctr);
	printf("locations 0: %d      1: %d      2: %d       3: %d      4: %d     5: %d \n",sqnc_locations[0],sqnc_locations[1],sqnc_locations[2],sqnc_locations[3],sqnc_locations[4],sqnc_locations[5]);
	free(script_ptr_first);
}

void mnlp_listener()
{
	unsigned char readData[175];
	unsigned char *readPtr = readData;
	int retValInt,readSize = 174;
	UARTbus bus = bus0_uart;
	f_enterFS();
	printf("entered listener\n");

	while (1)
	{
		kicktime(MNLPLISTENER_THREAD);
		// only if ADCS is turned on
		if (states & STATE_MNLP_ON)
		{
			retValInt = UART_read(bus, readData, readSize);
			readPtr = readData;
			if (readData[0] == 0x00)
			{
				readPtr++;
			}

			print_array(readPtr,readSize);
			if(readPtr[0]==0xBB)
			{
				mnlp_err= get_ERR_code(readPtr);
			}
			else//everything is good
			{
				if (readPtr[0]==0x09 || readPtr[0]==0x0A)
				{
					Time_getUnixEpoch(&timeout_mnlp);
					printf("return val is %d\n",retValInt);
				}
				Build_PayloadPacket(readPtr);
			}
		}
		vTaskDelay(500);
	}
}

void turn_on_payload()
{
	states |= STATE_MNLP_ON;
	Pin Pin04 = GPIO_04;
	Pin Pin05 = GPIO_05;
	Pin Pin06 = GPIO_06;
	Pin Pin07 = GPIO_07;
	PIO_Set(&Pin04);
	vTaskDelay(10);
	PIO_Set(&Pin05);
	vTaskDelay(10);
	PIO_Set(&Pin06);
	vTaskDelay(10);
	PIO_Set(&Pin07);
	vTaskDelay(10);
	FRAM_write(&states, STATES_ADDR, 1);
}
void turn_off_payload()
{
	Pin Pin04 = GPIO_04;
	Pin Pin05 = GPIO_05;
	Pin Pin06 = GPIO_06;
	Pin Pin07 = GPIO_07;
	states &= ~STATE_MNLP_ON;
	PIO_Clear(&Pin04);
	vTaskDelay(10);
	PIO_Clear(&Pin05);
	vTaskDelay(10);
	PIO_Clear(&Pin06);
	vTaskDelay(10);
	PIO_Clear(&Pin07);
	vTaskDelay(10);
	FRAM_write(&states, STATES_ADDR, 1);

}

char get_ERR_code(unsigned char * pct)
{
	char *msg[24];
	msg[0]= (char *){"IN RESET STATE CMD_ID 0x## IS A WRONG COMMAND"};
	msg[1]= (char *){"START CALIBRATION: WRONG COMMAND CMD_ID 0x## not equal SU_CAL (0x##)"};
	msg[2]= (char *){"START CALIBRATION: WRONG COMMAND LEN # not equal to 2"};
	msg[3]= (char *){"START CALIBRATION: WRONG CH#"};
	msg[4]= (char *){"Tried g_gpio_mLP_ch_enable data ready # times, then the function gives up"};
	msg[5]= (char *){"FAILED IN HEALTH CHECK"};
	msg[6]= (char *){"FAILED TO TURN PROBE BIAS ON"};
	msg[7]= (char *){"PROBE BIAS TURNED OFF"};
	msg[8]= (char *){"FAILED TO TURN PROBE BIAS OFF"};
	msg[9]= (char *){"FAILED TO TURN PROBE MTEE ON"};
	msg[10]= (char *){"MTEE TURNED OFF"};
	msg[11]= (char *){"FAILED TO TURN MTEE OFF"};
	msg[12]= (char *){"FAILED TO RUN CALIBRATION"};
	msg[13]= (char *){"FAILED IN HEALTH CHECK"};
	msg[14]= (char *){"FAILED TO SETUP HOUSEKEEPING"};
	msg[15]= (char *){"FAILED TO SETUP STM"};
	msg[16]= (char *){"MNLP IN UNKNOWN STATE"};
	msg[17]= (char *){"START SCIENCE: BIAS IS NOT ON"};
	msg[18]= (char *){"START SCIENCE: HEALTH CHECK IS NOT PERFORMED"};
	msg[19]= (char *){"START SCIENCE: WRONG COMMAND CMD_ID 0x## not equal SU_SCI (0x##)"};
	msg[20]= (char *){"START SCIENCE: WRONG COMMAND LEN # not equal to 3"};
	msg[21]= (char *){"LOAD PARAMETERS: WRONG COMMAND CMD_ID 0x## not equal SU_LDP (0x##)"};
	msg[22]= (char *){"LOAD PARAMETERS: WRONG COMMAND LEN # not equal to QB50_LEN_MAX (#)"};
	msg[23]= (char *){"setProbeBias_mNlp(#) error, only 0: OFF and 1: ON are allowed"};
	int len= pct[2];

	int i=0,j=0;
	int ret=-1;
	pct+=3;
	for(i=0;i<24;i++)
	{
		for(j=0;j<len;j++)
		{
			if(msg[i][j]!='#' && msg[i][j]!=pct[j]) break;
		}
		if(j==len-1) ret=i;
	}
	return ret;
}


void build_save_error_packets(int active_idx,unsigned char err)
{
	unsigned char mnlp_error_packet[MNLP_DATA_SIZE];
	short length;
	int i;

	mnlp_error_packet[0] = 0xFA;
	mnlp_error_packet[1] = 0x00;
	mnlp_error_packet[2] = err;


	// add header for active index
	// read length
	FRAM_read((unsigned char *)&length, scripts_adresses[active_idx], 2);

	// read xsum to mnlp_error_packet
	FRAM_read(&mnlp_error_packet[3], scripts_adresses[active_idx]+length-2, 2);

	// read header to place
	FRAM_read(&mnlp_error_packet[5], scripts_adresses[active_idx]+2, 10);



	// add rest of indices
	for (i = 0; i < NUM_SCRIPTS; i++)
	{
		// read length
		FRAM_read((unsigned char *)&length, scripts_adresses[i], 2);

		// read xsum to mnlp_error_packet
		FRAM_read(&mnlp_error_packet[15+i*12], scripts_adresses[i]+length-2, 2);

		// read header to place
		FRAM_read(&mnlp_error_packet[17+i*12], scripts_adresses[i]+2, 10);
	}

	Build_PayloadPacket(mnlp_error_packet);
}

void error_handle( )
{


	turn_off_payload();
	vTaskDelay(60000);
	turn_on_payload();
}
