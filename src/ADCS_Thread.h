/*
 * ADCS_Thread.h
 *
 *  Created on: 18 באפר 2016
 *      Author: USER1
 */

#ifndef ADCS_THREAD_H_
#define ADCS_THREAD_H_

#include "main.h"

typedef struct ADCS_comissioning_data
{
	int current_state;
	short estimated_anglar_rates[3];
	short magnetic_field_vactor[3];
	short sensor_rates[3];
	char RAW_CSS[6];
	short RAW_Magnetometer[3];
	short Magnetorquer_commands[3];
	short estimated_attitude_angles[3];
	short sattelite_position[3];
	short sattelite_velocity[3];
	short RAW_nadir_sensors[4];
	short RAW_sun_sensors[4];
	short nadir_centroid[2];
	short wheel_speed_estimation[3];
	short wheel_speed_command[3];
}ADCS_comissioning_data;

typedef struct ADCS_telemetry_data
{
	unsigned short csense_3v3curr;
	unsigned char csense_nadirSRAMcurr;
	unsigned char csense_sunSRAMcurr;
	unsigned short arm_cpuTemp;
	unsigned short ccontrol_3v3curr;
	unsigned short ccontrol_5Vcurr;
	unsigned short ccontrol_Vbatcurr;
	unsigned short magtorquer_curr;
	unsigned short momentum_wheelcurr;
	char ratesensor_temp;
	char magnetometer_temp;
}ADCS_telemetry_data;

typedef struct ADCS_Payload_Telametry
{
	short estimated_anglar_rates[3];
	short estimated_attitude_angles[3];
}ADCS_Payload_Telemetry;

extern int adcs_advance_stage;
extern int adcs_stage;

void task_adcs_commissioning();

void adc_stage(int stage);
void ADC_Stage_1();
void ADC_Stage_2();
void ADC_Stage_3();
void ADC_Stage_4();
void ADC_Stage_5();
void ADC_Stage_6();
void ADC_Stage_7();
void ADC_Stage_8();



#endif /* ADCS_THREAD_H_ */
