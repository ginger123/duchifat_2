/*
 * EPS.h
 *
 *  Created on: 21 αιπε 2016
 *      Author: LAVIAN
 */

#ifndef EPS_H_
#define EPS_H_

#include "main.h"
#include "GomEPS.h"
#include "FileSys.h"
#include "IsisTRXVU.h"
#ifndef EPS_address
#define EPS_address 0x02
#endif

#define EPS_TLM_SIZE 42
#define EPS_VOLTAGE_ADDR 0x1000
#define EPS_VOLTAGE_SIZE 6

typedef struct HKP_Struct {
	unsigned char sid;
	unsigned char HK_states;

	//EPS PARAM START
	unsigned short HK_vbatt;
	unsigned short HK_vboost[3];
	unsigned short HK_curin[3];
	short HK_temp[6];
	unsigned short HK_cursys;
	unsigned short HK_cursun;

	//EPS PARAM END

	//COMM START
	unsigned short HK_rx_doppler;
	unsigned short HK_rx_rssi;
	unsigned short HK_rx_bus_volt;
	unsigned short HK_tx_pa_temp;
	unsigned short HK_rx_lo_temp;
	unsigned short HK_tx_supply_curr;
	unsigned short HK_rx_supply_curr;
	unsigned char HK_rx_uptime[4];
	unsigned char HK_tx_uptime[4];
	unsigned short HK_tx_power_fwd_dbm;
	unsigned short HK_tx_power_refl_dbm;

	//COMM END

	//ANTENA START
	unsigned short HK_ants_temperature;
	unsigned short ant;
	//ANTENA END



} HK_Struct;


void EPS_Power_Conditioning(gom_eps_hk_t* EPS_Cur_TLM, unsigned short* Vbatt_Previous, gom_eps_channelstates_t* channels_state);
void EPS_Init(gom_eps_hk_t* EPS_Cur_TLM, gom_eps_channelstates_t *channels_state, unsigned short* vbatt_previous);
void Cruse(gom_eps_channelstates_t* channels_state);
void Safe(gom_eps_channelstates_t* channels_state);
void Write_F_EPS_TLM(gom_eps_hk_t* EPS_CUR_TLM);
void HK_packet_build_save(HK_Struct* Packet, gom_eps_hk_t tlm, ISIStrxvuRxTelemetry tlmRX, ISIStrxvuTxTelemetry tlmTX, ISISantsTelemetry antstlm);


#endif /* EPS_H_ */
