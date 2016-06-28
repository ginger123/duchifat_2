/*
 * main.c
 */

#include "main.h"

#define ENABLE_MAIN_TRACES 1
#if ENABLE_MAIN_TRACES
	#define MAIN_TRACE_INFO			TRACE_INFO
	#define MAIN_TRACE_DEBUG		TRACE_DEBUG
	#define MAIN_TRACE_WARNING		TRACE_WARNING
	#define MAIN_TRACE_ERROR		TRACE_ERROR
	#define MAIN_TRACE_FATAL		TRACE_FATAL
#else
	#define MAIN_TRACE_INFO(...)	{ }
	#define MAIN_TRACE_DEBUG(...)	{ }
	#define MAIN_TRACE_WARNING(...)	{ }
	#define MAIN_TRACE_ERROR		TRACE_ERROR
	#define MAIN_TRACE_FATAL		TRACE_FATAL
#endif

#ifndef NULL
#define NULL ( (void *) 0)
#endif

#ifndef FALSE
#define FALSE  0
#endif

#define TIME_ADDR 0x10BB
#define TIME_SIZE 8


global_param glb;


void initialize_satellite_time(Boolean deployed)
{
	unsigned char t[TIME_SIZE];
	unsigned long rt;
	initEpoch();
	if(!deployed)
	{
		Time_getUnixEpoch(&rt);
		FRAM_write((unsigned char*)&rt,TIME_ADDR,TIME_SIZE);
	}
	else
	{
		FRAM_read(t, TIME_ADDR,TIME_SIZE);
		printf("FRAM time: %c,%c,%c,%c,%c,%c,%c,%c\n",t[0],t[1],t[2],t[3],t[4],t[5],t[6],t[7]);
		int k = 1;
		int i;
		for (i=0;i<TIME_SIZE;i++)
		{
			rt = rt+t[7-i]*k;
			k*=256;
		}
		Time_setUnixEpoch(rt);
	}
}


void ants_init(void)
{

	ISISantsI2Caddress myAntennaAddress[2];
	myAntennaAddress[0].addressSideA = 0x31;
	myAntennaAddress[0].addressSideB = 0x32;

	//Initialize the AntS system
	IsisAntS_initialize(myAntennaAddress, 1);
}


void deploy_ants()
{
	IsisAntS_attemptDeployment(0,isisants_sideA,isisants_antenna1, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideA,isisants_antenna2, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideB,isisants_antenna3, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideB,isisants_antenna4, isisants_normalDeployment,10);
	
	//this stuff is the sattelite subsystem's way of deploying
	//ISISantsSide side =  isisants_sideA;
	/*unsigned char antennaSystemsIndex = 0;

		IsisAntS_setArmStatus(antennaSystemsIndex, side, isisants_disarm);

		vTaskDelay(5 / portTICK_RATE_MS);

		IsisAntS_setArmStatus(antennaSystemsIndex, side, isisants_arm);

		vTaskDelay(5 / portTICK_RATE_MS);

		IsisAntS_autoDeployment(antennaSystemsIndex, side, AUTO_DEPLOYMENT_TIME);*/

}


void initialize_subsystems(gom_eps_hk_t* EpsTelemetry_hk, gom_eps_channelstates_t *channels_state, unsigned short* vbatt_previous)
{

	Boolean deployed;
	//unsigned char adcs_reset = 1;

	// initialize I2C
	I2C_start(66000, 10);

	// initalize WDT
	WDT_startWatchdogKickTask(10 / portTICK_RATE_MS, FALSE);

	//initialize FRAM
	FRAM_start();

	//init antenna
	ants_init();

	//check if antenas deplyed
	deployed = check_ants_deployed();
	printf("deployed %d\n", (int)deployed);
	//initialize satellite time
	initialize_satellite_time(deployed);

	//initialize EPS
	if(!deployed)
	{
		unsigned char voltages[6] = {65,72,74,75,73,66};
		FRAM_write(voltages, EPS_VOLTAGE_ADDR, 6);
	}
	EPS_Init(EpsTelemetry_hk, channels_state, vbatt_previous);

	//initialize trxvu
	init_trxvu();
	IsisTrxvu_tcSetAx25Bitrate(0,trxvu_bitrate_1200);

	//initialize file system
	if(!deployed)
	{
		InitializeFS();
	}
	else
	{
		f_enterFS();
	}

	//initializing global parameters
	if(!deployed)
	{
		Set_Mute(TRUE);
		Set_Mnlp_State(FALSE);
	}
	else
	{
		Set_Mute(FALSE);
		Set_Mnlp_State(TRUE);
	}


	//initialize ADCS
	//ADCS_reset(&adcs_reset);
	//SetAttEstMode(2);
}


void taskMain()
{
		unsigned short vbatt_previous;
		//ADCS_CUR_STATE ADc;
		//ADc.flag = 0;
		gom_eps_channelstates_t channels_state;
		gom_eps_hk_t EpsTelemetry_hk;
		isisRXtlm rxtlm;
		unsigned long start_gs_time;
		unsigned long time_now_unix;
		unsigned long pt;
		Boolean deployed;


		// Initialize subsystems
		initialize_subsystems(&EpsTelemetry_hk, &channels_state, &vbatt_previous);

		AllinAll();

		deployed = check_ants_deployed();
		pt = FRAM_read((unsigned char *)&pt, TIME_ADDR, TIME_SIZE);
		printf("pt%lu\n",pt);
		Time_getUnixEpoch(&pt);
		printf("pt%lu\n",pt);
		while(!deployed)
		{
			GomEpsGetHkData_general(0, &EpsTelemetry_hk);
			EPS_Power_Conditioning(&EpsTelemetry_hk, &vbatt_previous, &channels_state);
			unsigned long rt;
			Time_getUnixEpoch(&rt);
			printf("rt%lu\n",rt);
			printf("waited for love for %lu seconds \n", rt-pt);
			if(rt - pt >= (unsigned long)10)
			{
				//deploy_ants();
				deployed = TRUE;
				pt = rt;
				FRAM_write((unsigned char *)&pt, TIME_ADDR, TIME_SIZE);
				Set_Mute(FALSE);
			}
			vTaskDelay(1000);
		}

		int counter = 0;
		unsigned long pt_beacon = pt;
		printf("love was given\n");
		while(1)
		{
			// 1. get telemetry trxvu
			vurc_getRxTelemTest(&rxtlm);

			// 2. get telemetry EPS
			GomEpsGetHkData_general(0, &EpsTelemetry_hk);

			// 3. Take unix time
			Time_getUnixEpoch(&time_now_unix);

			//printf("%d", EpsTelemetry_hk.fields.vbatt);
			if(counter>=5)
			{
				counter = 0;
				Write_F_EPS_TLM(&EpsTelemetry_hk);
			}
			// 3. get telemetry ADCS

			// 4. EPS power conditioning
			EPS_Power_Conditioning(&EpsTelemetry_hk, &vbatt_previous, &channels_state);

			trxvu_logic(&start_gs_time, &time_now_unix);

			counter++;
			// 6. mNLP
			//if(!(states & STATE_GS))
			//{
				// enter Mnlp code here!!!!
			//}
			if(time_now_unix - pt >= 60)
			{
				pt = time_now_unix;
				FRAM_write((unsigned char *)&time_now_unix,TIME_ADDR, TIME_SIZE);
			}
			if(time_now_unix - pt_beacon >= BACON_TIME)
			{
				Beacon(EpsTelemetry_hk);
			}
			vTaskDelay(2000 / portTICK_RATE_MS);
			//add data to files
		}
}



int main() {
	unsigned int i = 0;
	xTaskHandle taskMainHandle;
	xTaskHandle taskADCScomHandle;
	TRACE_CONFIGURE_ISP(DBGU_STANDARD, 2000000, BOARD_MCK);
	// Enable the Instruction cache of the ARM9 core. Keep the MMU and Data Cache disabled.
	CP15_Enable_I_Cache();

	printf("\n\r -- ISIS-OBC First Project Program Booted --\n\r");
	#ifdef __OPTIMIZE__
		printf("\n\r -- Compiled on  %s %s in release mode --\n\r", __DATE__, __TIME__);
	#else
		printf("\n\r -- Compiled on  %s %s in debug mode --\n\r", __DATE__, __TIME__);
	#endif
	// The actual watchdog is already started, this only initializes the watchdog-kick interface.
	WDT_start();
	printf("\t main: Starting main task.. \n\r");
	xTaskGenericCreate(taskMain, (const signed char*)"taskMain", 1024, NULL, configMAX_PRIORITIES-2, &taskMainHandle, NULL, NULL);
	//xTaskGenericCreate(task_adcs_commissioning, (const signed char*)"task_adcs_com", 1024, NULL, configMAX_PRIORITIES-2, &taskADCScomHandle, NULL, NULL);

	printf("\t main: Starting scheduler.. \n\r");
	vTaskStartScheduler();

	// This part should never be reached.
	printf("\t main: Waiting in an infinite loop. \n\r");
	while(1) {
		LED_wave(1);
		printf("MAIN: STILL ALIVE %d\n\r", i);
		i++;
	}

	return 0;
}
