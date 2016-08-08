/*
 * main.c
 */

#include "main.h"
#include "test.h"

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

#define AUTO_DEPLOYMENT_TIME 10

global_param glb;
xTaskHandle taskMNLPcomHandle,taskMNLPlistener,taskADCScomHandle,taskMainHandle,taskResetHandle;
xTaskHandle taskThreadCheck;
unsigned long timestamp[THREAD_TIMESTAMP_LEN];//0=main 1=mnlp 2=mnlplistener 3=adcs 4=reset

void task_reset();
void taskMain();

void kicktime(int n)
{
	unsigned long t;
	Time_getUnixEpoch(&t);
	timestamp[n]=t;
}
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
	//our guess
	/*IsisAntS_attemptDeployment(0,isisants_sideA,isisants_antenna1, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideA,isisants_antenna2, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideB,isisants_antenna3, isisants_normalDeployment,10);
	IsisAntS_attemptDeployment(0,isisants_sideB,isisants_antenna4, isisants_normalDeployment,10);*/

	//this stuff is the sattelite subsystem's way of deploying
		ISISantsSide side =  isisants_sideA;
		unsigned char antennaSystemsIndex = 0;
			IsisAntS_setArmStatus(antennaSystemsIndex, side, isisants_disarm);
			vTaskDelay(5 / portTICK_RATE_MS);
			IsisAntS_setArmStatus(antennaSystemsIndex, side, isisants_arm);
			vTaskDelay(5 / portTICK_RATE_MS);
			IsisAntS_autoDeployment(antennaSystemsIndex, side, AUTO_DEPLOYMENT_TIME);
}

void Initialize_UART()
{
	UARTconfig configBus0 = {.mode = AT91C_US_USMODE_NORMAL | AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS | AT91C_US_PAR_NONE | AT91C_US_OVER_16 | AT91C_US_NBSTOP_1_BIT,
									.baudrate = 9600, .timeGuard = 1, .busType = rs232_uart, .rxtimeout = 0x03C0};

	// Both UART peripherals must be started separately as they can use different configurations.
    UART_start(bus0_uart, configBus0);
}

void Initialized_GPIO()
{

	Pin Pin04 = GPIO_04;
	Pin Pin05 = GPIO_05;
	Pin Pin06 = GPIO_06;
	Pin Pin07 = GPIO_07;


	PIO_Configure(&Pin04, PIO_LISTSIZE(&Pin04));
	if(!PIO_Configure(&Pin04, PIO_LISTSIZE(Pin04))) {
			printf(" PinTest: Unable to configure PIOA pins as output! \n\r");
			while(1);
	}
	vTaskDelay(10);
	PIO_Configure(&Pin05, PIO_LISTSIZE(&Pin05));
	if(!PIO_Configure(&Pin05, PIO_LISTSIZE(Pin05))) {
			printf(" PinTest: Unable to configure PIOB pins as output! \n\r");
			while(1);
	}
	vTaskDelay(10);
	PIO_Configure(&Pin06, PIO_LISTSIZE(&Pin06));
	if(!PIO_Configure(&Pin06, PIO_LISTSIZE(Pin06))) {
		printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
		while(1);
	}
	vTaskDelay(10);
	PIO_Configure(&Pin07, PIO_LISTSIZE(&Pin07));
	if(!PIO_Configure(&Pin07, PIO_LISTSIZE(Pin06))) {
			printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
			while(1);
	}
	printf("\n\r PinTest: All pins should now be logic-0 (0V). Please check their states now. \n\r");
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
		unsigned char voltages[6] = {66,72,74,75,73,67}; //
		FRAM_write(voltages, EPS_VOLTAGE_ADDR, EPS_VOLTAGE_SIZE);
	}
	EPS_Init(EpsTelemetry_hk, channels_state, vbatt_previous);

	//initialize trxvu
	init_trxvu();
	IsisTrxvu_tcSetAx25Bitrate(0,trxvu_bitrate_1200);

	// initialize UART
	Initialize_UART();

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

	Initialized_GPIO();
	//initialize ADCS
	//ADCS_reset(&adcs_reset);
	//SetAttEstMode(2);
}

void task_threadkeeper()
{
	unsigned long t;
	while(1)
	{
		Time_getUnixEpoch(&t);
		if(t > timestamp[0]+THREAD_TIMEOUT )//0=main 1=mnlp 2=mnlplistener 3=adcs 4=reset
		{
			printf("---THREAD MAIN FAILED. RESTARTING ALL---\n");
			vTaskDelete(taskMainHandle);
			vTaskDelete(taskMNLPcomHandle);
			vTaskDelete(taskMNLPlistener);
			vTaskDelete(taskADCScomHandle);
			vTaskDelete(taskResetHandle);
			gracefulReset();
			//xTaskGenericCreate(taskMain, (const signed char*)"taskMain", 1024, NULL, configMAX_PRIORITIES-2, &taskMainHandle, NULL, NULL);
			//kicktime(MAIN_THREAD);
		}
		if(t > timestamp[1]+THREAD_TIMEOUT )
		{
			printf("---THREAD MNLP FAILED. RESTARTING---\n");
			vTaskDelete(taskMNLPcomHandle);
			xTaskGenericCreate(taskmnlp, (const signed char*)"taskMnlp", 1024, NULL, configMAX_PRIORITIES-2, &taskMNLPcomHandle, NULL, NULL);
			kicktime(MNLP_THREAD);

		}
		if(t > timestamp[2]+THREAD_TIMEOUT )
		{
			printf("---THREAD MNLP LISTENER FAILED. RESTARTING---\n");
			vTaskDelete(taskMNLPlistener);
			xTaskGenericCreate(mnlp_listener, (const signed char*)"taskMnlp", 1024, NULL, configMAX_PRIORITIES-2, &taskMNLPlistener, NULL, NULL);
			kicktime(MNLPLISTENER_THREAD);
		}
		if(t > timestamp[3]+THREAD_TIMEOUT )
		{
			printf("---THREAD ADCS FAILED. RESTARTING---\n");
			vTaskDelete(taskADCScomHandle);
			xTaskGenericCreate(task_adcs_commissioning, (const signed char*)"task_adcs_com", 1024, NULL, configMAX_PRIORITIES-2, &taskADCScomHandle, NULL, NULL);
			kicktime(ADCS_THREAD);
		}
		if(t > timestamp[4]+THREAD_TIMEOUT )
		{
			printf("---THREAD RESET FAILED. RESTARTING---\n");
			vTaskDelete(taskResetHandle);
			xTaskGenericCreate(task_reset, (const signed char*)"taskReset", 1024, NULL, configMAX_PRIORITIES-2, &taskResetHandle, NULL, NULL);
			kicktime(RESET_THREAD);
		}
		vTaskDelay(THREAD_TIMEOUT);
	}
}
void task_reset()
{
	int i=0;
	for(i=0;i<600;i++)
		{
		//printf("reset in %d seconds\n",30-i);
		vTaskDelay(1000);
		kicktime(RESET_THREAD);
		}
	gracefulReset();
	while(1);
}

void taskMain()
{
	unsigned short vbatt_previous;

	//ADCS_CUR_STATE ADc;
	//ADc.flag = 0;
	gom_eps_channelstates_t channels_state;
	gom_eps_hk_t EpsTelemetry_hk;
	isisRXtlm rxtlm;
	ISIStrxvuRxTelemetry rx_tlm;
	ISIStrxvuTxTelemetry tx_tlm;
	ISISantsTelemetry ants_tlm;
	unsigned long start_gs_time;
	unsigned long time_now_unix;
	unsigned long pt;
	int j;
	Boolean deployed;

	if (0) // only for cubesupport
	{
		WDT_startWatchdogKickTask(10 / portTICK_RATE_MS, FALSE);
		I2C_start(66000, 10);
		printf("EPS initializing\n");
		EPS_Init(&EpsTelemetry_hk, &channels_state, &vbatt_previous);
		printf("Done\n");

		while (1)
		{
			vTaskDelay(500);
		}
	}

	// Initialize subsystems
	initialize_subsystems(&EpsTelemetry_hk, &channels_state, &vbatt_previous);
	for(j=0;j<THREAD_TIMESTAMP_LEN;j++)
	{
		kicktime(j);
	}
	deployed = check_ants_deployed();

	FRAM_read((unsigned char *)&pt, TIME_ADDR, TIME_SIZE);
	//initialize reset thread which resets every x seconds the satellite
	xTaskGenericCreate(task_reset, (const signed char*)"taskReset", 1024, NULL, configMAX_PRIORITIES-2, &taskResetHandle, NULL, NULL);
	while(!deployed)
	{
		GomEpsGetHkData_general(0, &EpsTelemetry_hk);
		EPS_Power_Conditioning(&EpsTelemetry_hk, &vbatt_previous, &channels_state);
		unsigned long rt;
		Time_getUnixEpoch(&rt);
		printf("rt%lu\n",rt);
		printf("waited for love for %lu seconds \n", rt-pt);
		if(rt - pt >= (unsigned long)5)
		{
			kicktime(MAIN_THREAD);
			//deploy_ants();
			deployed = TRUE;
			pt = rt;
			FRAM_write((unsigned char *)&pt, TIME_ADDR, TIME_SIZE);
			Set_Mute(FALSE);
		}
		vTaskDelay(1000);
	}

	unsigned long pt_beacon = pt;
	unsigned long pt_hk = pt;
	printf("love was given\n");
	//AllinAll();
	xTaskGenericCreate(taskmnlp, (const signed char*)"taskMnlp", 1024, NULL, configMAX_PRIORITIES-2, &taskMNLPcomHandle, NULL, NULL);
	xTaskGenericCreate(mnlp_listener, (const signed char*)"taskMnlp", 1024, NULL, configMAX_PRIORITIES-2, &taskMNLPlistener, NULL, NULL);
	xTaskGenericCreate(task_adcs_commissioning, (const signed char*)"task_adcs_com", 1024, NULL, configMAX_PRIORITIES-2, &taskADCScomHandle, NULL, NULL);
	xTaskGenericCreate(task_threadkeeper, (const signed char*)"taskThread", 1024, NULL, configMAX_PRIORITIES-2, &taskThreadCheck, NULL, NULL);

	while(1)
	{
		kicktime(MAIN_THREAD);
		// 1. get telemetry trxvu
		//vurc_getRxTelemTest(&rxtlm);

		IsisTrxvu_rcGetTelemetryAll(0, &rx_tlm);

		IsisTrxvu_tcGetTelemetryAll(0,&tx_tlm);

		IsisAntS_getAlltelemetry(0, isisants_sideA, &ants_tlm);

		// 2. get telemetry EPS
		GomEpsGetHkData_general(0, &EpsTelemetry_hk);

		// 3. Take unix time
		Time_getUnixEpoch(&time_now_unix);

		if(time_now_unix - pt_hk >= 10)
		{
			printf("sending HK packets\n");
			pt_hk = time_now_unix;
			//HK_packet_build_save(EpsTelemetry_hk,rx_tlm,tx_tlm,ants_tlm);
			eslADCS_telemetry_Time_Power_temp();
			//idtlm();
			printf("local time: %lu bat volt %d    states  %x\n",time_now_unix,EpsTelemetry_hk.fields.vbatt, states);
		}
		// 3. get telemetry ADCS

		// 4. EPS power conditioning
		EPS_Power_Conditioning(&EpsTelemetry_hk, &vbatt_previous, &channels_state);

		trxvu_logic(&start_gs_time, &time_now_unix);

		// 6. mNLP
		//if(!(states & STATE_GS))
		//{
		// enter Mnlp code here!!!!
		//}
		if(time_now_unix - pt >= 70)
		{
			pt = time_now_unix;
			FRAM_write((unsigned char *)&time_now_unix,TIME_ADDR, TIME_SIZE);

			// check tasks operation
			check_task(&taskMNLPcomHandle,taskmnlp, (const signed char*)"taskMnlp", NULL);
			check_task(&taskMNLPlistener,mnlp_listener, (const signed char*)"taskMnlp", NULL);
			check_task(&taskADCScomHandle,task_adcs_commissioning, (const signed char*)"task_adcs_com", NULL);

		}
		if(time_now_unix - pt_beacon >= BACON_TIME)
		{
			//dumpparam[0] = 3;
			//convert_time_array(time_now_unix-60,&dumpparam[1]);
			//convert_time_array(time_now_unix+10,&dumpparam[6]);
			//dump(dumpparam);


			pt_beacon = time_now_unix;

			//Beacon(EpsTelemetry_hk);
		}


		vTaskDelay(2000 / portTICK_RATE_MS);
		//add data to files
	}
}



int main() {
	unsigned int i = 0;
	//

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

void check_task(xTaskHandle *handle,pdTASK_CODE task, const signed char *name, void *pvParameters)
{
	eTaskState task_state = eTaskGetState(handle);
	if(task_state == eSuspended)
	{
		vTaskDelete(*handle);
		xTaskGenericCreate(task, name, 1024, pvParameters, configMAX_PRIORITIES-2, handle, NULL, NULL);
	}
}
