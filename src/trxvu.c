/*
 * trxvu.c
 *
 *  Created on: 10 ???? 2016
 *      Author: Ariel
 */

#include "main.h"
#include "EPS.h"
#include "mnlp.h"
unsigned char tc_count;
unsigned char dumpparam[11];
unsigned int dump_completed = 0;
unsigned int dump_created = 0;
unsigned char beacon_count=0;
xTaskHandle taskDumpHandle;
unsigned long last_wod;
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

void act_upon_comm(unsigned char* in, unsigned short length)
{

	unsigned int i=0;
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
			if(decode.srvc_subtype==125)//dump
			{
				tc_verification_report(decode,TC_EXEC_START_SUCCESS,NO_ERR,in);

				//dump(decode.data[0],decode.data[1]);
				int i = 0;
				for(;i<11;i++)
				{
					dumpparam[i] = decode.data[i];
				}
				if (dump_created==0)
				{
					xTaskGenericCreate(dump, (const signed char*)"taskDump", 1024, (void *)&dumpparam[0], configMAX_PRIORITIES-2, &taskDumpHandle, NULL, NULL);
					dump_created=1;
				}

			}
			if(decode.srvc_subtype==132)//delete packets function
			{



				unsigned long time_to_del = (decode.data[1]<<24) + (decode.data[2]<<16) + (decode.data[3]<<8) + (decode.data[4]);
				unsigned int storid_to_del= (int)decode.data[0];
				print_file("HK_packets",HK_SIZE+5);
				delete_packets_from_file(storid_to_del, time_to_del);
				print_file("HK_packets",HK_SIZE+5);
				//call function for deletion


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
				if(decode.srvc_subtype==134)//reset FTW
				{
					//change this to whatever reset sequence we need
					gracefulReset();
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
				printf("save segment %d",decode.data[0]);
				FRAM_write(decode.data+1,SCRIPT_RAW_ADDR+BLOCK_SIZE*decode.data[0],decode.len-1);
			}
			if(decode.srvc_subtype==132)//upload last frame
			{
				// read length and script
				short len;
				unsigned char *script_ptr;
				FRAM_read((unsigned char *)&len, SCRIPT_RAW_ADDR, 2);
				script_ptr = (unsigned char *)calloc(len,sizeof(char));
				FRAM_read(script_ptr, SCRIPT_RAW_ADDR, len);

				// check CRC
				if(Fletcher16(script_ptr,len)) printf("bad script");
				else{
				printf("received full script\n printing it now\n");
				print_array(script_ptr,len);				// write to appropriate address

				FRAM_write(script_ptr,scripts_adresses[decode.data[0]],len);//save script numbe
				}
				// free memory
				free(script_ptr);
			}
			if(decode.srvc_subtype==133)//delete script
			{
				unsigned char sc_num=decode.data[0];
				unsigned char* cc = (unsigned char*) calloc(200,sizeof(char));
				memset(cc,0xFF,200);
				FRAM_write(cc,scripts_adresses[sc_num],200);

				FRAM_read(cc,scripts_adresses[sc_num],200);
				print_array(cc,200);
				free(cc);

			}
			if(decode.srvc_subtype==133)
			{
				if(decode.data[0])
				{
					states |= STATE_MNLP_ON_GROUND;
				}
				else
				{
					states &= ~STATE_MNLP_ON_GROUND;
				}
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

				int j;
				for(j=0;j<THREAD_TIMESTAMP_LEN;j++)
				{
					timestamp[j]=t;
				}
				Time_setUnixEpoch(t);
				ADCS_update_unix_time(t);
			}
			if(decode.srvc_subtype==2)
			{
				ADCS_update_tle(decode.data);
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
			if(decode.srvc_subtype == 3)
			{
				unsigned char n_voltages[EPS_VOLTAGE_SIZE];
				int i = 0;
				for(;i<6;i++)
				{
					n_voltages[i] = decode.data[i];
				}
				FRAM_write(n_voltages, EPS_VOLTAGE_ADDR, EPS_VOLTAGE_SIZE);
				vTaskDelay(1);
				unsigned char voltages[EPS_VOLTAGE_SIZE];
				FRAM_read(voltages,EPS_VOLTAGE_ADDR, EPS_VOLTAGE_SIZE);
				print_array(n_voltages,6);
				print_array(voltages,6);
			}
		break;
		default:
			free(decode.data);
			tc_count++;
			FRAM_write(&tc_count,TC_COUNT_ADDR,1);
			tc_verification_report(decode,TC_EXEC_COMPLETE_FAILURE,ERR_ILEGAL_DATA,in);//ERROR NO COMMAND RECOGNIZED
		return;
	}

	tc_verification_report(decode,TC_EXEC_COMPLETE_SUCCESS,NO_ERR,in);
	free(decode.data);
	tc_count++;
	FRAM_write(&tc_count,TC_COUNT_ADDR,1);
	printf("exited act upon command\n");
}

void dump(void *arg)
{
	int ret;
	ret = f_enterFS(); /* Register this task with filesystem */
	ASSERT( (ret == F_NO_ERROR ), "f_enterFS pb: %d\n\r", ret);

	//dump_created = 1;

	if(!Get_Mute() && !(states & STATE_MUTE_EPS))
	{
		unsigned char type = 0;unsigned long start_time = 0; unsigned long final_time = 0;
		unsigned char* argument = (unsigned char*)arg;
		printf("entered dump\n");
		start_time = convert_epoctime((char *)&argument[1]);
		final_time=convert_epoctime((char *)(&argument[6]));
		type=argument[0];
		printf("type is %d, start time is %lu end time is %lu\n",type,start_time,final_time);
		if(start_time>final_time)
		{
			dump_completed = 1;
			printf("SumTing Iz FckeD Up\n");
			while(1)
			{
				vTaskDelay(5000);
			}
		}
		char HK_packets[] = {"HK_packets"};
		char ADC_comm[] = {"adcs_file"};
		char ADC_tlm[] = {"adcs_tlm_file"};
		char mnlp_file[] ={"mnlp"};
		char wod_file[] ={"wod_file"};
		char *file;
		unsigned char *temp_data;
		int size=0;
		int i=0;
		int start_idx = 0;
		int num_packets = 0;
		unsigned long t_l;
		ccsds_packet pct;
		int end_offest;


		pct.srvc_type=3;
		pct.srvc_subtype=25;

		switch (type)
		{
			case 1://dumping HK packet packet
				file=HK_packets;
				size=HK_SIZE;
				printf("dump HK\n");
				end_offest = 2;
				break;
			case 2://dumping adcs commisionning
				file= ADC_comm;
				size= ADC_COMM_SIZE;
				printf("dump adc commisioning\n");
				break;
			case 3:
				file=ADC_tlm;
				end_offest = 12;
				size = sizeof(ADCS_telemetry_data);
				printf("dump adc_tlm\n");
				break;
			case 4:
				pct.srvc_type=130;
				pct.srvc_subtype=1;
				file = mnlp_file;
				end_offest = sizeof(ADCS_Payload_Telemetry)+MNLP_DATA_SIZE;
				size = sizeof(ADCS_Payload_Telemetry)+MNLP_DATA_SIZE;
				printf("dump mnlp\n");
				break;
			case 5:
				file = wod_file;
				end_offest = 9;
				size = 9;
				printf("dump WOD\n");
				break;
			default:
				return;
				break;
		}

		pct.len=size;//updates data based on the type of the packet
		pct.apid=10;
		temp_data = (unsigned char*)calloc(size+5,sizeof(char));

		num_packets = find_number_of_packets(file,size+5,start_time,final_time,&start_idx);

		printf("sending %d packets\n",num_packets);

		for(i=0;i<num_packets;i++)//sending packets
		{
			printf("loop counter %d\n",i);
			FileReadIndex(file, (char *)temp_data,size+5,start_idx+i);
			print_array(temp_data,size+5);
			// convert time to epoch time
			t_l = convert_epoctime((char *) temp_data);
			t_l = t_l -30*365*24*3600-24*3600*7;
			convert_time_array(t_l, pct.c_time);

			pct.data = temp_data+5;

			switch_endian(pct.data + end_offest, size - end_offest);

			//delete_packets_from_file(file, todel,size);
			send_SCS_pct(pct);

			printf("sent packet %d\n",i);
		}

		free(temp_data);

	}
	dump_completed = 1;
	while (1)
	{
		vTaskDelay(500 / portTICK_RATE_MS);
	}

}



void enter_gs_mode(unsigned long *start_gs_time)
{
	printf("enter ground station mode");
	// Enter ground station mode
	states |= STATE_GS;

	//Disable Mnlp
	//glb_channels_state.fields.channel3V3_2 = 0;
	//glb_channels_state.fields.channel5V_2 = 0;
	//GomEpsSetOutput(0, glb_channels_state);

	//Sets the initial time of the pass
	Time_getUnixEpoch(start_gs_time);
}

void end_gs_mode()
{
	printf("exit ground station mode");
	//Exit ground station mode
	states &= ~STATE_GS;
	//initialize Mnlp
	//glb_channels_state.fields.channel3V3_2 = 1;
	//glb_channels_state.fields.channel5V_2 = 1;
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
	//printf("rx_counter is %d\n",RxCounter);
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
			act_upon_comm(receive_frm,rxFrameCmd.rx_length);
		}
		Time_getUnixEpoch(time_now_unix);
		if(*time_now_unix - *start_gs_time >= GS_TIME)
		{
			end_gs_mode();
		}
	}

	// check if need to delete dump command
	if ((dump_created==1) && (dump_completed==1))
	{
		dump_created = 0;
		dump_completed = 0;
		vTaskDelete( taskDumpHandle );
		printf("delete dump task\n");
	}

}

void Beacon(gom_eps_hk_t EpsTelemetry_hk)
{
	ccsds_packet beacon;
	unsigned char dat[9];
	unsigned short telemetryValue;
	unsigned long cur_wod;
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

	Set_Vbatt(EpsTelemetry_hk.fields.vbatt);
	Set_Cursys(EpsTelemetry_hk.fields.cursys);
	Set_Curout3V3(EpsTelemetry_hk.fields.curout[4]);
	Set_Curout5V(EpsTelemetry_hk.fields.curout[0]);
	Set_tempCOMM(eng_value);
	Set_tempEPS(EpsTelemetry_hk.fields.temp[0]);
	Set_tempBatt(EpsTelemetry_hk.fields.temp[4]);

	if(states & (STATE_MNLP_ON_EPS | STATE_MNLP_ON_GROUND)) dat[1]=1;
	else dat[1]=0;
	dat[2]=glb.vbatt;
	dat[3]=glb.cursys;
	dat[4]=glb.curout3V3;
	dat[5]=glb.curout5V;
	dat[6]=glb.tempCOMM;
	dat[7]=glb.tempEPS;
	dat[8]=glb.tempBatt;

	Time_getUnixEpoch(&cur_wod);
	if(cur_wod-last_wod>60)//of more than 60 sec elapsed after last save. save wod again
	{
		WritewithEpochtime("wod_file",0,dat,9);
		last_wod=cur_wod;
	}




	if(!Get_Mute())
	{

		printf("send beacon\n");
		update_time(beacon.c_time);

		if(beacon_count%3==0)
		{
			beacon_count=0;
			IsisTrxvu_tcSetAx25Bitrate(0,trxvu_bitrate_1200);

		}
		else
		{
			IsisTrxvu_tcSetAx25Bitrate(0,trxvu_bitrate_9600);
		}
		beacon_count++;
		send_SCS_pct(beacon);

		IsisTrxvu_tcSetAx25Bitrate(0,trxvu_bitrate_9600);
	}

}
