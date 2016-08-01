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
}



void EPS_Init(gom_eps_hk_t* EPS_Cur_TLM, gom_eps_channelstates_t *channels_state, unsigned short* vbatt_previous)
{
	unsigned char EPS_addr = EPS_address;
	eps_config_t eps_config;
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

	if (0)
	{
	GomEpsConfigGet(0, &eps_config);
	//printf("\n\n\nHeater low is: %d\n",eps_config.fields.battheater_low);
	printf("Heater high is: %d\n",eps_config.fields.battheater_high);
	printf("mode is: %d\n\n\n",eps_config.fields.battheater_mode);
	eps_config.fields.battheater_low = 0;
	eps_config.fields.battheater_high = 10;
	eps_config.fields.battheater_mode = 1;
	GomEpsConfigSet(0,&eps_config);
	GomEpsConfigGet(0, &eps_config);
	printf("\n\n\nHeater low is: %d\n",eps_config.fields.battheater_low);
	printf("Heater high is: %d\n",eps_config.fields.battheater_high);
	printf("mode is: %d\n\n\n",eps_config.fields.battheater_mode);
	}
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


void HK_packet_build_save(gom_eps_hk_t tlm, ISIStrxvuRxTelemetry tlmRX, ISIStrxvuTxTelemetry tlmTX, ISISantsTelemetry antstlm)
{
	HK_Struct Packet;
	char sd_file_name[] = {"HK_packets"};
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

	// send packet
	ccsds_packet ccs_packet;
	ccs_packet.apid=10;
	ccs_packet.srvc_type = 3;
	ccs_packet.srvc_subtype = 25;
	update_time(ccs_packet.c_time);
	ccs_packet.len = sizeof(HK_Struct);
	ccs_packet.data = (unsigned char*)&Packet;


	//send_SCS_pct(ccs_packet);
}
