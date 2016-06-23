/*
 * trxvu.c

 *
 *  Created on: 10 ???? 2016
 *      Author: Ariel
 */

#include "main.h"
#include "EPS.h"
unsigned char tc_count;
unsigned char dumpparam[2];

void update_wod(gom_eps_hk_t EpsTelemetry_hk)
{
	Set_Vbatt(EpsTelemetry_hk.fields.vbatt);
	Set_Cursys(EpsTelemetry_hk.fields.cursys);
	Set_Curout3V3(EpsTelemetry_hk.fields.curout[0]);
	Set_Curout5V(EpsTelemetry_hk.fields.curout[3]);
	Set_tempCOMM(25);
	Set_tempEPS((short)EpsTelemetry_hk.fields.temp[0]);
	Set_tempBatt((short)EpsTelemetry_hk.fields.temp[4]);
}

void vurc_getRxTelemTest(isisRXtlm *converted)
{
	unsigned short telemetryValue;
	float eng_value = 0.0;
	ISIStrxvuRxTelemetry telemetry;

	// Telemetry values are presented as raw values
	//printf("\r\nGet all Telemetry at once in raw values \r\n\r\n");
	IsisTrxvu_rcGetTelemetryAll(0, &telemetry);

	telemetryValue = telemetry.fields.tx_current;
	eng_value= ((float)telemetryValue) * 0.0897;
	converted->tx_current=eng_value;
	//printf("Transmitter current is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.rx_doppler;
	eng_value = ((float)telemetryValue) * 6.837 - 14000;
	converted->rx_doppler=eng_value;
	//printf("Receiver doppler value is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.rx_current;
	eng_value = ((float)telemetryValue) * 0.0305;
	converted->rx_current=eng_value;
	//printf("Receiver current is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.bus_volt;
	eng_value = ((float)telemetryValue) * 0.00488;
	converted->bus_volt=eng_value;
	//printf("Bus voltage is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.board_temp;
	eng_value = ((float)telemetryValue) * -0.0546 + 189.5522;
	converted->board_temp=eng_value;
	//printf("Board temperature = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.pa_temp;
	eng_value = ((float)telemetryValue) * -0.0546 + 189.5522;
	converted->pa_temp=eng_value;
	//printf("Pa temperature = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.rx_rssi;
	eng_value = ((float)telemetryValue * 0.03) + 152;
	converted->rx_rssi=eng_value;
	//printf("Receiver RSSI = %f \r\n", eng_value);


}

 void vurc_getTxTelemTest(isisTXtlm *converted)
{
	unsigned short telemetryValue;
	float eng_value = 0.0;
	ISIStrxvuTxTelemetry telemetry;

	/*
	float tx_reflpwr; ///< Tx Telemetry reflected power.
	        float pa_temp; ///< Tx Telemetry power amplifier temperature.
	        float tx_fwrdpwr; ///< Tx Telemetry forward power.
	        float tx_current; ///< Tx Telemetry transmitter current.
	*/
	// Telemetry values are presented as raw values
	//printf("\r\nGet all Telemetry at once in raw values \r\n\r\n");
	IsisTrxvu_tcGetTelemetryAll(0, &telemetry);

	telemetryValue = telemetry.fields.tx_current;
	eng_value = ((float)telemetryValue) * 0.0897;
	converted->tx_current=eng_value;
	//printf("Transmitter current is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.pa_temp;
	eng_value = ((float)telemetryValue) * -0.0546 + 189.5522;
	converted->pa_temp=eng_value;
	//printf("PA temperature is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.tx_reflpwr;
	eng_value = ((float)(telemetryValue * telemetryValue)) * 0.00005887;
	converted->tx_reflpwr=eng_value;
	//printf("RF reflected power is = %f \r\n", eng_value);

	telemetryValue = telemetry.fields.tx_fwrdpwr;
	eng_value = ((float)(telemetryValue * telemetryValue)) * 0.00005887;
	converted->tx_fwrdpwr=eng_value;
	//printf("RF reflected power is = %f \r\n", eng_value);


}

 void init_trxvu(void)
 {
     int retValInt = 0;

     // Definition of I2C and TRXUV
 	ISIStrxvuI2CAddress myTRXVUAddress[1];
 	ISIStrxvuFrameLengths myTRXVUBuffers[1];
 	ISIStrxvuBitrate myTRXVUBitrates[1];

 	//I2C addresses defined
 	myTRXVUAddress[0].addressVu_rc = 0x60;
 	myTRXVUAddress[0].addressVu_tc = 0x61;

 	//Buffer definition
 	myTRXVUBuffers[0].maxAX25frameLengthTX = SIZE_TXFRAME;
 	myTRXVUBuffers[0].maxAX25frameLengthRX = SIZE_RXFRAME;

 	//Bitrate definition
 	myTRXVUBitrates[0] = trxvu_bitrate_1200;

 	if(retValInt != 0)
 	{
 		printf("\n\r I2Ctest: I2C_start_Master for TRXUV test: %d! \n\r", retValInt);
 	}

 	//Initialize the trxvu subsystem
 	IsisTrxvu_initialize(myTRXVUAddress, myTRXVUBuffers, myTRXVUBitrates, 1);
 }

int TRX_sendFrame(unsigned char* data, unsigned char length)
{
	unsigned char avalFrames=0;
	IsisTrxvu_tcSendAX25DefClSign(0, data, length, &avalFrames);
	if(avalFrames==0) return -1;
	availableFrames=avalFrames;
	//printf("\navailable space in queue: %d\n",availableFrames);
	return 0;
}

void act_upon_comm(unsigned char* in)
{

	unsigned int i=0;
	unsigned int script_len;
	FRAM_read(&tc_count,TC_COUNT_ADDR,1);
	in++;
	rcvd_packet decode;
	parse_comm(&decode,in);
	//if(!decode.isvalidcrc) return;
	printf("\n-----packet structure start-----\n");
	printf("\napid is: %d",decode.apid);
	printf("\nlength is: %d",decode.len);
	printf("\nservice type is: %d",decode.srvc_type);
	printf("\nservice subtype is: %d\n",decode.srvc_subtype);
	printf("data: ");
	for(i=0;i<decode.len;i++)
	{
		printf("%02x ",decode.data[i]);
	}
	printf("\n-----packet structure end-----\n");

	switch(decode.srvc_type)
	{
		case (3):
			if(decode.srvc_subtype==131)//dump
			{
				//tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);
				xTaskHandle taskDumpHandle;
				//dump(decode.data[0],decode.data[1]);
				dumpparam[0] = decode.data[0];
				dumpparam[1] = decode.data[1];
				printf("Command Dump params %x %x\n",dumpparam[0],dumpparam[1]);
				xTaskGenericCreate(dump, (const signed char*)"taskDump", 1024, (void *)&dumpparam[0], configMAX_PRIORITIES-2, &taskDumpHandle, NULL, NULL);
				vTaskDelay(5000 / portTICK_RATE_MS);

			}
		break;
		case (8):
				printf("service type 8");
				if(decode.srvc_subtype==131)
				{
					tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);
					Set_Mute(TRUE);//enable mute
					printf("Command Mute\n");
				}
				if(decode.srvc_subtype==132)
				{
					tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);
					Set_Mute(FALSE);//disable mute
					printf("Command Disable Mute\n");
				}
				if(decode.srvc_subtype==137)
				{
					adcs_stage=decode.data[0];
					printf("Change stage to %d\n",adcs_stage);
				}
			break;
		case (130):
			if(decode.srvc_subtype==131)//upload subpacket
			{
				FRAM_write(decode.data+1,SCRIPT_RAW_ADDR+BLOCK_SIZE*decode.data[0],BLOCK_SIZE);
			}
			if(decode.srvc_subtype==132)//upload last frame
			{
				printf("received full script\n");
				unsigned char printscript[500];
				FRAM_write(decode.data,SCRIPT_RAW_ADDR-1,1);//save script number
				FRAM_write(decode.data+1,SCRIPT_RAW_ADDR-3,2);//save script size
				script_len=decode.data[1]*256+decode.data[2];

				FRAM_read(printscript,SCRIPT_RAW_ADDR,script_len);
				for(i=0;i<script_len;i++)
				{
					printf("%x ", printscript[i]);
				}
				printf("\n");
				//invoke function here
			}
		break;
		case (131):
			if(decode.srvc_subtype==1)//update time. may be subject to change in servive type /subtypr
			{
				printf("Command Set Time\n");
				tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);
				unsigned long t=0;
				t=decode.data[0];
				t=t<<8;
				t+=decode.data[1];
				t=t<<8;
				t+=decode.data[2];
				t=t<<8;
				t+=decode.data[3];
				printf("\nunix time is: %ld\n",t);
				Time_setUnixEpoch(t);

			}
			if(decode.srvc_subtype==4)
			{
				tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);
				ccsds_packet response;
				response.apid=10;
				response.srvc_type=9;
				response.srvc_subtype=4;
				response.len=0;
				update_time(response.c_time);
				send_SCS_pct(response);
				printf("Command report Time\n");
			}
			if(decode.srvc_type == 3)
			{
				unsigned char n_voltages[EPS_VOLTAGE_SIZE];
				int i = 0;
				for(;i<6;i++)
				{
					n_voltages[i] = decode.data[i];
				}
				FRAM_write(n_voltages, EPS_VOLTAGE_ADDR, EPS_VOLTAGE_SIZE);
			}
		break;
		default:
			free(decode.data);
			tc_count++;
			FRAM_write(&tc_count,TC_COUNT_ADDR,1);
			tc_verification_report(decode,TC_EXEC_COMPLETE_FAILURE,ERR_ILEGAL_DATA,in);//ERROR NO COMMAND RECOGNIZED
		return;
	}

	//tc_verification_report(decode,TC_EXEC_COMPLETE_SUCCESS,NO_ERR,in);
	free(decode.data);
	tc_count++;
	FRAM_write(&tc_count,TC_COUNT_ADDR,1);
}

void dump(void *arg)
{
	int ret;
	ret = f_enterFS(); /* Register this task with filesystem */
	ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);

	while (1)
	{
		vTaskDelay(2000 / portTICK_RATE_MS);
		AllinAll();
	}

	if(!Get_Mute())
	{
		unsigned char type;unsigned char am;
		unsigned char* argument = (unsigned char*)arg;
		printf("entered dump\n");
		type=argument[0];
		am=argument[1];
		printf("type is:%d amount is: %d\n",type,am);
		int todel[1]={0};
		char eps_file[] = {"EPSFILE"};//{'E','P','S','F','I','L','E'};
		char *file;
		int size=0;
		int i=0;
		ccsds_packet pct;

		pct.srvc_type=3;
		pct.srvc_subtype=25;
		type=1;
		if(type==1)//dumping eps
		{
			pct.len=EPS_TLM_SIZE+1;
			pct.apid=10;
			pct.data=(unsigned char*)calloc(EPS_TLM_SIZE+1,sizeof(char));
			pct.data[0]=0x05;
			file=eps_file;
			size=EPS_TLM_SIZE;
			printf("dump eps\n");
			// filename
		}
		else if(type==0x02)
		{
		}
		else if(type==0x03)
		{
		}
		for(;i<am;i++)
		{
			printf("sent packet %d\n",i);
			FileRead(eps_file ,(char *)pct.data+1, size);
			//delete_packets_from_file(file, todel,size);
			update_time(pct.c_time);
			send_SCS_pct(pct);

			vTaskDelay(500 / portTICK_RATE_MS);
		}

		free(pct.data);
	}
}



void enter_gs_mode(unsigned long *start_gs_time)
{
	printf("enter ground station mode");
	// Enter ground station mode
	states = states | STATE_GS;

	//Disable Mnlp
	glb_channels_state.fields.channel3V3_2 = 0;
	glb_channels_state.fields.channel5V_2 = 0;
	//GomEpsSetOutput(0, glb_channels_state);

	//Sets the initial time of the pass
	Time_getUnixEpoch(start_gs_time);
}

void end_gs_mode()
{
	printf("exit ground station mode");
	//Exit grout station mode
	states = states & !(STATE_GS);
	//initialize Mnlp
	glb_channels_state.fields.channel3V3_2 = 1;
	glb_channels_state.fields.channel5V_2 = 1;
	//GomEpsSetOutput(0, glb_channels_state);
}

Boolean check_ants_deployed()// NOT WORKING CAUSE ISIS CODE
{
	/*ISISantsSide side = isisants_sideA;
	ISISantsStatus ants_stat;

	side = isisants_sideA;
	IsisAntS_getStatusData(0,side,&ants_stat);
	side = isisants_sideB;
	IsisAntS_getStatusData(0,side,&ants_stat);
	printf("%d,%d,%d,%d\n",ants_stat.fields.ant1Undeployed,ants_stat.fields.ant2Undeployed,ants_stat.fields.ant3Undeployed,ants_stat.fields.ant4Undeployed);
	if(ants_stat.fields.ant1Undeployed || ants_stat.fields.ant2Undeployed || ants_stat.fields.ant3Undeployed || ants_stat.fields.ant4Undeployed)
	{
		printf("deployed\n");
		return TRUE;
	}*/
	printf("not deployed\n");
	return FALSE;

}


void trxvu_logic(unsigned long *start_gs_time, unsigned long *time_now_unix)
{
	unsigned char receive_frm[SIZE_RXFRAME];
	ISIStrxvuRxFrame rxFrameCmd = {0,0,0, receive_frm};
	unsigned short RxCounter=0;
	int i=0;
	// 5. check command receive
	IsisTrxvu_rcGetFrameCount(0, &RxCounter);
	printf("\n rxcounter is: %d\n",RxCounter);
	// 6. check if begin GS mode
	if(RxCounter>0)
	{
		if(!((states & STATE_GS) == STATE_GS))
		{
			enter_gs_mode(start_gs_time);
		}
	}

	if((states & STATE_GS) == STATE_GS)
	{
		IsisTrxvu_rcGetFrameCount(0, &RxCounter);
		if(RxCounter>0)
		{
			IsisTrxvu_rcGetCommandFrame(0, &rxFrameCmd);
			printf("RECEIVED PACKET\r\n");
			for(i=0;i<rxFrameCmd.rx_length;i++)
			{
				printf("%02x ", receive_frm[i]);

			}
			printf("\n\rEND RECEIVED PACKET\r\n");
			act_upon_comm(receive_frm);
		}
		Time_getUnixEpoch(time_now_unix);
		if(*time_now_unix - *start_gs_time >= GS_TIME)
		{
			end_gs_mode();
		}
	}
}

void Beacon(gom_eps_hk_t EpsTelemetry_hk)
{
	ccsds_packet beacon;
	unsigned char dat[9];
	unsigned short telemetryValue;
	float eng_value = 0.0;
	ISIStrxvuRxTelemetry telemetry;

	// Telemetry values are presented as raw values
	//printf("\r\nGet all Telemetry at once in raw values \r\n\r\n");
	IsisTrxvu_rcGetTelemetryAll(0, &telemetry);
	telemetryValue = telemetry.fields.board_temp;
	eng_value = ((float)telemetryValue) * -0.0546 + 189.5522;

	dat[0]=0xFF;
	beacon.data=dat;
	beacon.apid=10;
	beacon.srvc_type=3;
	beacon.srvc_subtype=25;
	beacon.len=9;
	update_time(beacon.c_time);
	if(!Get_Mute() && !((states & STATE_GS) == STATE_GS))
	{
		update_time(beacon.c_time);
		Set_Vbatt(EpsTelemetry_hk.fields.vbatt);
		Set_Cursys(EpsTelemetry_hk.fields.cursys);
		Set_Curout3V3(EpsTelemetry_hk.fields.curout[4]);
		Set_Curout5V(EpsTelemetry_hk.fields.curout[0]);
		Set_tempCOMM(eng_value);
		Set_tempEPS(EpsTelemetry_hk.fields.temp[0]+60);
		Set_tempBatt(EpsTelemetry_hk.fields.temp[4]+60);
		dat[1]= glb.Mnlp_State;
		dat[2]=glb.vbatt;
		dat[3]=glb.cursys;
		dat[4]=glb.curout3V3;
		dat[5]=glb.curout5V;
		dat[6]=glb.tempCOMM;
		dat[7]=glb.tempEPS;
		dat[8]=glb.tempBatt;

		send_SCS_pct(beacon);
	}
}
