/*
 * ADCS_Thread.h
 *
 *  Created on: 18 באפר 2016
 *      Author: USER1
 */

#ifndef ADCS_THREAD_H_
#define ADCS_THREAD_H_



typedef struct ADCS_comissioning_data
{
	char sid;
	char stage;
	char RAW_CSS[6];
	short estimated_anglar_rates[3];
	short magnetic_field_vactor[3];
	short sensor_rates[3];
	short RAW_Magnetometer[3];
	short Magnetorquer_commands[3];
	short estimated_attitude_angles[3];
	short sattelite_position[3];
	short sattelite_velocity[3];
	short RAW_nadir_sensors[4];
	short RAW_sun_sensors[4];
	short wheel_speed_estimation[3];
	short wheel_speed_command[3];
}ADCS_comissioning_data;


extern int adcs_advance_stage;
extern int adcs_stage,adcs_stage_param;

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
void adcs_stage_9();



#endif /* ADCS_THREAD_H_ */
