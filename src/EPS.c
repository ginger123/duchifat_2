#include "main.h"
#include "IsisTRXVU.h"

unsigned char states;


void EPS_Power_Conditioning(gom_eps_hk_t* EPS_Cur_TLM, unsigned short* Vbatt_Previous, gom_eps_channelstates_t* channels_state)
{
	unsigned char voltages[EPS_VOLTAGE_SIZE];
	FRAM_read(voltages, EPS_VOLTAGE_ADDR,EPS_VOLTAGE_SIZE);
	if(EPS_Cur_TLM->fields.vbatt < *Vbatt_Previous) // in case of battery discharge
	{
		if(EPS_Cur_TLM->fields.vbatt < (int)voltages[0]*100 && channels_state->fields.channel3V3_1 == 1)
		{
			Safe(channels_state);
			states &= ~STATE_ADCS_ON_EPS;
			GomEpsSetOutput(0, *channels_state); // Shuts down the ADCS actuators as well
		}
		else if(EPS_Cur_TLM->fields.vbatt < (int)voltages[1]*100)
		{
			Cruse(channels_state);
			states |= STATE_MUTE_EPS;
			GomEpsSetOutput(0, *channels_state); // Shuts down the transmitter as well

		}
		else if(EPS_Cur_TLM->fields.vbatt < (int)voltages[2]*100)
		{
			Cruse(channels_state);
			states &= ~STATE_MNLP_ON_EPS;
			GomEpsSetOutput(0, *channels_state); // Shuts down the payload
		}
	}
	else if(EPS_Cur_TLM->fields.vbatt > *Vbatt_Previous)
	{
		if(EPS_Cur_TLM->fields.vbatt > (int)voltages[3]*100)
		{
			Cruse(channels_state);
			states |= STATE_MNLP_ON_EPS;
			GomEpsSetOutput(0, *channels_state); // Activates the payload as well
		}
		else if(EPS_Cur_TLM->fields.vbatt > (int)voltages[4]*100)
		{
			Cruse(channels_state);
			states &= ~STATE_MUTE_EPS;
			GomEpsSetOutput(0, *channels_state); // Activates the tranciever as well
		}
		else if(EPS_Cur_TLM->fields.vbatt > (int)voltages[5]*100 && channels_state->fields.channel3V3_1 == 0)
		{
			Cruse(channels_state);
			states |= STATE_ADCS_ON_EPS;
			GomEpsSetOutput(0, *channels_state); // Activates the ADCS actuators
		}
	}
	*Vbatt_Previous = EPS_Cur_TLM->fields.vbatt;
	FRAM_write(&states, STATES_ADDR, 1);
}



void EPS_Init(gom_eps_hk_t* EPS_Cur_TLM, gom_eps_channelstates_t *channels_state, unsigned short* vbatt_previous)
{
	unsigned char EPS_addr = EPS_address;
	GomEpsInitialize(&EPS_addr, 1);
	GomEpsGetHkData_general(0, EPS_Cur_TLM);
	unsigned char voltages[EPS_VOLTAGE_SIZE];
	FRAM_read(voltages, EPS_VOLTAGE_ADDR,EPS_VOLTAGE_SIZE);

	if(EPS_Cur_TLM->fields.vbatt < voltages[0]*100)
	{
		Safe(channels_state);
		GomEpsSetOutput(0, *channels_state); // Shuts down the ADCS actuators as well
		states &= ~(STATE_ADCS_ON_EPS + STATE_MNLP_ON_EPS);
		states |= STATE_MUTE_EPS;
	}
	else if(EPS_Cur_TLM->fields.vbatt < voltages[1]*100)
	{
		printf("ADCS ON\n");
		Cruse(channels_state);
		GomEpsSetOutput(0, *channels_state); // Shuts down the transmitter as well
		states &= ~(STATE_MNLP_ON_EPS);
		states |= STATE_ADCS_ON_EPS + STATE_MUTE_EPS;
	}
	else if(EPS_Cur_TLM->fields.vbatt < voltages[2]*100)
	{
		printf("ADCS ON\n");
		Cruse(channels_state);
		GomEpsSetOutput(0, *channels_state); // Shuts down the payload
		states &= ~STATE_MNLP_ON_EPS;
		states |= STATE_ADCS_ON_EPS;
		states &= ~STATE_MUTE_EPS;
	}
	else
	{
		printf("ADCS ON\n");
		Cruse(channels_state);
		GomEpsSetOutput(0, *channels_state); // everything is on
		states |= STATE_ADCS_ON_EPS + STATE_MNLP_ON_EPS;
		states &= ~STATE_MUTE_EPS;
	}

	*vbatt_previous = EPS_Cur_TLM->fields.vbatt;
	FRAM_write(&states, STATES_ADDR, 1);
}

void print_config(eps_config_t config_data)
{
    printf(" battery low is %d and high is %d\n",(int)config_data.fields.battheater_low,(int)config_data.fields.battheater_high);
    printf(" heater mode %d\n",(int) config_data.fields.battheater_mode);
    printf("vboost PPT %d,%d,%d\n ", config_data.fields.vboost[0],config_data.fields.vboost[1],config_data.fields.vboost[2]);
}

void set_heater_values(char heater_params[2])
{
    eps_config_t config_data;
    GomEpsConfigGet(0,&config_data);
    print_array((unsigned char *)heater_params,2);
    printf("before:\n");
    print_config(config_data);
    config_data.fields.battheater_low = heater_params[0];
    config_data.fields.battheater_high = heater_params[1];
    GomEpsConfigSet(0,&config_data);

    GomEpsConfigGet(0,&config_data);
    printf("after:\n");
    print_config(config_data);
}

void Cruse(gom_eps_channelstates_t* channels_state)
{
	channels_state->fields.quadbatSwitch = 0;
	channels_state->fields.quadbatHeater = 0;
	channels_state->fields.channel3V3_1 = 1;
	channels_state->fields.channel3V3_2 = 0;
	channels_state->fields.channel3V3_3 = 0;
	channels_state->fields.channel5V_1 = 1;
	channels_state->fields.channel5V_2 = 0;
	channels_state->fields.channel5V_3 = 0;
}

void Safe(gom_eps_channelstates_t* channels_state)
{
	channels_state->fields.quadbatSwitch = 0;
	channels_state->fields.quadbatHeater = 0;
	channels_state->fields.channel3V3_1 = 0;
	channels_state->fields.channel3V3_2 = 0;
	channels_state->fields.channel3V3_3 = 0;
	channels_state->fields.channel5V_1 = 0;
	channels_state->fields.channel5V_2 = 0;
	channels_state->fields.channel5V_3 = 0;
}

void print_general_hk_packet(HK_Struct Packet)
{
	// print all values
	unsigned long rt;
	Time_getUnixEpoch(&rt);
	printf("---HOUSKEEPING AT TIME %lu--- \n",rt);
	printf("battery voltage %d\n mV",Packet.HK_vbatt);
	printf("Boost converters: %d,%d,%d mV\n", Packet.HK_vboost[0],Packet.HK_vboost[1],Packet.HK_vboost[2]);
	printf("Currents: %d,%d,%d mA\n", Packet.HK_curin[0],Packet.HK_curin[1],Packet.HK_curin[2]);
	printf("temperature %d, %d, %d, %d, %d, %d C\n",Packet.HK_temp[0],Packet.HK_temp[1],Packet.HK_temp[2],Packet.HK_temp[3],Packet.HK_temp[4],Packet.HK_temp[5]);
	printf("current out of battery %d mA\n",Packet.HK_cursys);
	printf("current sun sensor %d mA\n",Packet.HK_cursun);
	printf("states %x mA\n",Packet.HK_states);

	// receiver
	printf("doppler offset %f kHz\n",Packet.HK_rx_doppler*13.352-22300);
	printf("RSSI %f dBm\n",Packet.HK_rx_rssi*0.03-152);
	printf("Bus voltage %f V\n",Packet.HK_rx_bus_volt*0.00488);
	printf("local oscilator temp %f C\n",Packet.HK_rx_lo_temp*-0.0546+189.5522);
	printf("Rx up time %d days %d  hours %dminutes %d seconds\n", Packet.HK_rx_uptime[3],Packet.HK_rx_uptime[2],Packet.HK_rx_uptime[1],Packet.HK_rx_uptime[0]);
	printf("Rx suppply current %f mA\n",Packet.HK_rx_supply_curr*0.0305);

	// transceiver
	printf("Power amp temp %f C\n",Packet.HK_tx_pa_temp*-0.0546+189.5522);
	printf("Tx up time %d days %d  hours %dminutes %d seconds\n", Packet.HK_rx_uptime[3],Packet.HK_rx_uptime[2],Packet.HK_rx_uptime[1],Packet.HK_tx_uptime[0]);
	printf("Tx suppply current %f mA\n",Packet.HK_tx_supply_curr*0.0305);

	//antennas
	printf("antennas temp: %f C\n",(Packet.HK_ants_temperature * -0.2922) + 190.65);
	printf("antennas deployment status %x\n",Packet.ant);
}

void HK_packet_build_save(gom_eps_hk_t tlm, ISIStrxvuRxTelemetry tlmRX, ISIStrxvuTxTelemetry tlmTX, ISISantsTelemetry antstlm)
{
	HK_Struct Packet;
	char sd_file_name[] = {"HK_packets"};
	int end_offset = 2;
	int size = HK_SIZE;

	Packet.sid= EPS_SID;
	//EPS PARAM START
	Packet.HK_vbatt = tlm.fields.vbatt;
	Packet.HK_vboost[0] = tlm.fields.vboost[0];
	Packet.HK_vboost[1] = tlm.fields.vboost[1];
	Packet.HK_vboost[2] = tlm.fields.vboost[2];
	Packet.HK_curin[0] = tlm.fields.curin[0];
	Packet.HK_curin[1] = tlm.fields.curin[1];
	Packet.HK_curin[2] = tlm.fields.curin[2];
	Packet.HK_temp[0] = tlm.fields.temp[0];
	Packet.HK_temp[1] = tlm.fields.temp[1];
	Packet.HK_temp[2] = tlm.fields.temp[2];
	Packet.HK_temp[3] = tlm.fields.temp[3];
	Packet.HK_temp[4] = tlm.fields.temp[4];
	Packet.HK_temp[5] = tlm.fields.temp[5];
	Packet.HK_cursys = tlm.fields.cursys;
	Packet.HK_cursun = tlm.fields.cursun;
	Packet.HK_states = states;
	//EPS PARAM END

				//COMM START
	//RX
	Packet.HK_rx_doppler = tlmRX.fields.rx_doppler;
	Packet.HK_rx_rssi = tlmRX.fields.rx_rssi;
	Packet.HK_rx_bus_volt = tlmRX.fields.bus_volt;
	Packet.HK_rx_lo_temp = tlmRX.fields.board_temp;
	Packet.HK_rx_supply_curr = tlmRX.fields.rx_current;
	IsisTrxvu_rcGetUptime(0,Packet.HK_rx_uptime);

	//TX
	Packet.HK_tx_pa_temp = tlmTX.fields.pa_temp;
	Packet.HK_tx_supply_curr = tlmTX.fields.tx_current;
	Packet.HK_tx_power_fwd_dbm = tlmTX.fields.tx_reflpwr;
	Packet.HK_tx_power_refl_dbm = tlmTX.fields.tx_fwrdpwr;
	IsisTrxvu_tcGetUptime(0,Packet.HK_tx_uptime);

	//Antena
	Packet.HK_ants_temperature = antstlm.fields.ants_temperature;
	Packet.ant = antstlm.fields.ants_deployment.raw[1]*256 + antstlm.fields.ants_deployment.raw[0];
				//COMM END
	WritewithEpochtime(sd_file_name, 0,(char *)&Packet,sizeof(HK_Struct));

	//printf("Vbatt is %d\n Vboosts are %d,%d,%d\n Curin are %d,%d,%d\n EPS temps are %d,%d,%d,%d,%d,%d\n, cursys is %d\n, cursun is %d\n states is %d\n rx doppler is %d\n rx rssi is %d\n",Packet.HK_vbatt,Packet.HK_vboost[0],Packet.HK_vboost[1],Packet.HK_vboost[2],Packet.HK_curin[0],Packet.HK_curin[1],Packet.HK_curin[2],Packet.HK_temp[0],Packet.HK_temp[1],Packet.HK_temp[2],Packet.HK_temp[3],Packet.HK_temp[4],Packet.HK_temp[5],Packet.HK_cursys,Packet.HK_cursun,Packet.HK_states,Packet.HK_rx_doppler,Packet.HK_rx_rssi);
	//printf("rx bus volt is %d\n rx lo temp is %d\n rx supply curr is %d\n tx pa temp is %d\n tx supply curr is %d\n",Packet.HK_rx_bus_volt,	Packet.HK_rx_lo_temp,Packet.HK_rx_supply_curr,Packet.HK_tx_pa_temp,Packet.HK_tx_supply_curr);
	//printf("tx power fwb is %d\n tx power refl is %d\n ants temp is %d\n, ant is %d\n",Packet.HK_tx_power_fwd_dbm,Packet.HK_tx_power_refl_dbm,Packet.HK_ants_temperature,Packet.ant);


	// send packet
	ccsds_packet ccs_packet;
	ccs_packet.apid=10;
	ccs_packet.srvc_type = 3;
	ccs_packet.srvc_subtype = 25;
	update_time(ccs_packet.c_time);
	ccs_packet.len = sizeof(HK_Struct);
	ccs_packet.data = (unsigned char*)&Packet;

	print_general_hk_packet(Packet);

	switch_endian(ccs_packet.data + end_offset, size - end_offset);
	//send_SCS_pct(ccs_packet);
}
