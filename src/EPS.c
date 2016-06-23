#include "main.h"
#include "IsisTRXVU.h"

void EPS_Power_Conditioning(gom_eps_hk_t* EPS_Cur_TLM, unsigned short* Vbatt_Previous, gom_eps_channelstates_t* channels_state)
{
	unsigned char voltages[EPS_VOLTAGE_SIZE];
	FRAM_read(voltages, EPS_VOLTAGE_ADDR,EPS_VOLTAGE_SIZE);
	if(EPS_Cur_TLM->fields.vbatt < *Vbatt_Previous) // in case of battery discharge
	{
		if(EPS_Cur_TLM->fields.vbatt < (int)voltages[0]*100 && channels_state->fields.channel3V3_1 == 1)
		{
			Safe(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Shuts down the ADCS actuators as well
		}
		else if(EPS_Cur_TLM->fields.vbatt < (int)voltages[1]*100 && channels_state->fields.channel5V_1 == 1)
		{
			Cruse(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Shuts down the transmitter as well

		}
		else if(EPS_Cur_TLM->fields.vbatt < (int)voltages[2]*100 && channels_state->fields.channel5V_3 == 1)
		{
			Cruse(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Shuts down the payload
		}
	}
	else
	{
		if(EPS_Cur_TLM->fields.vbatt > (int)voltages[3]*100 && channels_state->fields.channel5V_3 == 0)
		{
			Cruse(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Activates the payload as well
		}
		else if(EPS_Cur_TLM->fields.vbatt > (int)voltages[4]*100 && channels_state->fields.channel5V_1 == 0)
		{
			Cruse(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Activates the tranciever as well
		}
		else if(EPS_Cur_TLM->fields.vbatt > (int)voltages[5]*100 && channels_state->fields.channel3V3_1 == 0)
		{
			Cruse(channels_state);
			//GomEpsSetOutput(EPS_address, channels_state); // Activates the ADCS actuators
		}
	}
	*Vbatt_Previous = EPS_Cur_TLM->fields.vbatt;
	/*if((EPS_Cur_TLM->fields.curout[0] > 500 || EPS_Cur_TLM->fields.curout[1] > 500 || EPS_Cur_TLM->fields.curout[2] > 500 || EPS_Cur_TLM->fields.curout[3] > 500 || EPS_Cur_TLM->fields.curout[4] > 500 || EPS_Cur_TLM->fields.curout[5] > 500) && (COMPONENT_On_Off&Over_Current_bit) == 0)
	{
		COMPONENT_On_Off = COMPONENT_On_Off | (Over_Current_bit); //Over current state
	}
	else if((COMPONENT_On_Off & Over_Current_bit) == Over_Current_bit)
	{
		COMPONENT_On_Off = COMPONENT_On_Off & (No_Over_Current); // Leaving over current state
	}
	if((EPS_Cur_TLM->fields.temp[0]> 69 || EPS_Cur_TLM->fields.temp[1]> 69 || EPS_Cur_TLM->fields.temp[2]> 69 || EPS_Cur_TLM->fields.temp[3]> 69 || EPS_Cur_TLM->fields.temp[4]>69 ||EPS_Cur_TLM->fields.temp[5]> 69) && (COMPONENT_On_Off&Over_Heat_bit) == 0)
	{
		COMPONENT_On_Off = COMPONENT_On_Off | (Over_Heat_bit);
	}
	else if((COMPONENT_On_Off & Over_Heat_bit) == Over_Heat_bit)
	{
		COMPONENT_On_Off = COMPONENT_On_Off & (No_Over_Heat);
	}*/
}



void EPS_Init(gom_eps_hk_t* EPS_Cur_TLM, gom_eps_channelstates_t *channels_state, unsigned short* vbatt_previous)
{
	unsigned char EPS_addr = EPS_address;
	GomEpsInitialize(&EPS_addr, 1);
	GomEpsGetHkData_general(EPS_address, EPS_Cur_TLM);
	unsigned char voltages[EPS_VOLTAGE_SIZE];
	FRAM_read(voltages, EPS_VOLTAGE_ADDR,EPS_VOLTAGE_SIZE);

	if(EPS_Cur_TLM->fields.vbatt < voltages[0]*100)
	{
		Safe(channels_state);
		GomEpsSetOutput(EPS_address, *channels_state); // Shuts down the ADCS actuators as well
	}
	else if(EPS_Cur_TLM->fields.vbatt < voltages[1]*100)
	{
		Cruse(channels_state);
		GomEpsSetOutput(EPS_address, *channels_state); // Shuts down the transmitter as well
	}
	else if(EPS_Cur_TLM->fields.vbatt < voltages[2]*100)
	{
		Cruse(channels_state);
		GomEpsSetOutput(EPS_address, *channels_state); // Shuts down the payload
	}
	else
	{
		Cruse(channels_state);
		GomEpsSetOutput(EPS_address, *channels_state); // everything is on
	}
	*vbatt_previous = EPS_Cur_TLM->fields.vbatt;
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

void Write_F_EPS_TLM(gom_eps_hk_t* EPS_CUR_TLM)
{
	char eps_tlm[EPS_TLM_SIZE];
	eps_tlm[0] = EPS_CUR_TLM->fields.vbatt/256;
	eps_tlm[1] = EPS_CUR_TLM->fields.vbatt%256;
	eps_tlm[2] = EPS_CUR_TLM->fields.vboost[0]/256;
	eps_tlm[3] = EPS_CUR_TLM->fields.vboost[0]%256;
	eps_tlm[4] = EPS_CUR_TLM->fields.vboost[1]/256;
	eps_tlm[5] = EPS_CUR_TLM->fields.vboost[1]%256;
	eps_tlm[6] = EPS_CUR_TLM->fields.vboost[2]/256;
	eps_tlm[7] = EPS_CUR_TLM->fields.vboost[2]%256;
	eps_tlm[8] = EPS_CUR_TLM->fields.curin[0]/256;
	eps_tlm[9] = EPS_CUR_TLM->fields.curin[0]%256;
	eps_tlm[10] = EPS_CUR_TLM->fields.curin[1]/256;
	eps_tlm[11] = EPS_CUR_TLM->fields.curin[1]%256;
	eps_tlm[12] = EPS_CUR_TLM->fields.curin[2]/256;
	eps_tlm[13] = EPS_CUR_TLM->fields.curin[2]%256;
	eps_tlm[14] = EPS_CUR_TLM->fields.curout[0]/256;
	eps_tlm[15] = EPS_CUR_TLM->fields.curout[0]%256;
	eps_tlm[16] = EPS_CUR_TLM->fields.curout[1]/256;
	eps_tlm[17] = EPS_CUR_TLM->fields.curout[1]%256;
	eps_tlm[18] = EPS_CUR_TLM->fields.curout[2]/256;
	eps_tlm[19] = EPS_CUR_TLM->fields.curout[2]%256;
	eps_tlm[20] = EPS_CUR_TLM->fields.curout[3]/256;
	eps_tlm[21] = EPS_CUR_TLM->fields.curout[3]%256;
	eps_tlm[22] = EPS_CUR_TLM->fields.curout[4]/256;
	eps_tlm[23] = EPS_CUR_TLM->fields.curout[4]%256;
	eps_tlm[24] = EPS_CUR_TLM->fields.curout[5]/256;
	eps_tlm[25] = EPS_CUR_TLM->fields.curout[5]%256;
	eps_tlm[26] = (EPS_CUR_TLM->fields.temp[0]+60)/256;  // temperatures will be + 60 deg c
	eps_tlm[27] = (EPS_CUR_TLM->fields.temp[1]+60)/256;
	eps_tlm[29] = (EPS_CUR_TLM->fields.temp[1]+60)%256 ;
	eps_tlm[30] = (EPS_CUR_TLM->fields.temp[2]+60)/256;
	eps_tlm[31] = (EPS_CUR_TLM->fields.temp[2]+60)%256 ;
	eps_tlm[32] = (EPS_CUR_TLM->fields.temp[3]+60)/256;
	eps_tlm[33] = (EPS_CUR_TLM->fields.temp[3]+60)%256 ;
	eps_tlm[34] = (EPS_CUR_TLM->fields.temp[4]+60)/256;
	eps_tlm[35] = (EPS_CUR_TLM->fields.temp[4]+60)%256 ;
	eps_tlm[36] = (EPS_CUR_TLM->fields.temp[5]+60)/256;
	eps_tlm[37] = (EPS_CUR_TLM->fields.temp[5]+60)%256;
	eps_tlm[38] = EPS_CUR_TLM->fields.cursys/256;
	eps_tlm[39] = EPS_CUR_TLM->fields.cursys%256;
	eps_tlm[40] = EPS_CUR_TLM->fields.cursun/256;
	eps_tlm[41] = EPS_CUR_TLM->fields.cursun%256;
	char EPS_File[] ={"EPSFILE"};
	FileWrite(EPS_File, 0, eps_tlm, EPS_TLM_SIZE);
}

void HK_packet_build_save(gom_eps_hk_t tlm, ISIStrxvuRxTelemetry tlmRX, ISIStrxvuTxTelemetry tlmTX, ISISantsTelemetry antstlm)
{
	HK_Struct Packet;
	char sd_file_name[] = {"HK_packets"};
	Packet.sid=166;
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
	FileWrite(sd_file_name, 0,(char *)&Packet,sizeof(HK_Struct));
}
