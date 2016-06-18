#include "main.h"

#ifndef NULL
#define NULL ( (void *) 0)
#endif

#ifndef FALSE
#define FALSE  0
#endif



void eslADCS_setStateADCS(adcs_state_t current_status)
{
	printf("set state\n");
	unsigned char arr[1];
	arr[0] = (char)current_status;
	ADCS_command(3 ,arr ,1);
}

void eslADCS_setEstimationMode(adcs_estmode_t mode)
{
	unsigned char arr;
	arr = (char)mode;
	ADCS_command(17, &arr,1);
}

void eslADCS_setPwrCtrlDevice(adcs_powerdev_t device_ctrl)
{
	unsigned char arr[5];                  // Import All The enums to one array
		arr[0]=device_ctrl.fields.signal_cubecontrol;
		arr[1]=device_ctrl.fields.motor_cubecontrol;
		arr[2]=device_ctrl.fields.pwr_cubesense;
		arr[3]=device_ctrl.fields.pwr_motor;
		arr[4]=device_ctrl.fields.pwr_gpsantlna;
		ADCS_command(5,arr,5);
}

void eslADCS_setAttitudeCtrlMode(adcs_ctrlmodeset_t modesettings)
{
	unsigned char arr[3];
	arr[0] = modesettings.fields.mode;
	arr[1] = modesettings.fields.timeout & 0xff;
	arr[2] = (modesettings.fields.timeout >> 8) & 0xff;
	ADCS_command(18, arr, 3);
}

void eslADCS_deployMagnetometer(unsigned char act_timeout)
{
	unsigned char arr;
	arr = act_timeout;
	ADCS_command(6, &arr, 1);
}

void eslADCS_setConfSave()
{
	unsigned char arr[1];
	arr[0] = 0;
	ADCS_command(100, arr, 1);
}

void eslADCS_setWheelSpeed(adcs_wheelspeed_t cmd_speed)
{
	unsigned char arr[6];
	arr[0] = cmd_speed.fields.speedX & 0xff;
	arr[1] = (cmd_speed.fields.speedX >> 8) & 0xff;
	arr[2] = cmd_speed.fields.speedY & 0xff;
	arr[3] = (cmd_speed.fields.speedY >> 8) & 0xff;
	arr[4] =cmd_speed.fields.speedZ & 0xff;
	arr[5] = (cmd_speed.fields.speedY >> 8) & 0xff;
	ADCS_command(32, arr, 6);
}

void eslADCS_getEstimatedAngRates(adcs_angrate_t* ang_rates)
{
		unsigned char data[6];
		unsigned char comm= 146;
		I2C_write(0x12,&comm,1);
		//ADCS_command(comm, NULL, 0);
		vTaskDelay(50 / portTICK_RATE_MS);

		I2C_read(0x12,data,6);

		ang_rates->fields.x_angrate = data[0]<<8;
		ang_rates->fields.x_angrate += data[1];
		ang_rates->fields.y_angrate = data[2]<<8;
		ang_rates->fields.y_angrate += data[3];
		ang_rates->fields.z_angrate = data[4]<<8;
		ang_rates->fields.z_angrate += data[5];
}

void eslADCS_getSensorRates(adcs_angrate_t* sen_rates)
{
	unsigned char data[6];
	unsigned char comm = 153;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	vTaskDelay(50 / portTICK_RATE_MS);

	I2C_read(0x12,data,6);

	sen_rates->fields.x_angrate = data[0]<<8;
	sen_rates->fields.x_angrate += data[1];
	sen_rates->fields.y_angrate = data[2]<<8;
	sen_rates->fields.y_angrate += data[3];
	sen_rates->fields.z_angrate = data[4]<<8;
	sen_rates->fields.z_angrate += data[5];
}

void eslADCS_getMagneticFieldVec(adcs_magfieldvec_t* mag_field)
{
	unsigned char data[6];
	unsigned char comm = 149;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	vTaskDelay(50 / portTICK_RATE_MS);


	I2C_read(0x12,data,6);

	mag_field->fields.x_magfield = data[0]<<8;
	mag_field->fields.x_magfield += data[1];
	mag_field->fields.y_magfield = data[2]<<8;
	mag_field->fields.y_magfield += data[3];
	mag_field->fields.z_magfield = data[4]<<8;
	mag_field->fields.z_magfield += data[5];
}

void eslADCS_getMagnetorquerCmd(adcs_magnetorq_t* mag_cmd)
{
	int i=0;
	unsigned char data[6];
	unsigned char comm = 155;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12,data,6);
	printf("magnetotorquer data from adc: ");
	for(i=0;i<6;i++)
	{
		printf("%x ",data[i]);
	}
	printf("\n");
	mag_cmd->fields.magX = data[0]<<8;
	mag_cmd->fields.magX += data[1];
	mag_cmd->fields.magY = data[2]<<8;
	mag_cmd->fields.magY += data[3];
	mag_cmd->fields.magZ = data[4]<<8;
	mag_cmd->fields.magZ += data[5];
}

void eslADCS_getWheelSpeed(adcs_wheelspeed_t* wheel_speed)
{
	unsigned char data[6];
	unsigned char comm = 154;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 1);
	I2C_read(0x12, data, 6);
	wheel_speed->fields.speedX = data[0]<<8;
	wheel_speed->fields.speedX += data[1];
	wheel_speed->fields.speedY = data[2]<<8;
	wheel_speed->fields.speedY += data[3];
	wheel_speed->fields.speedZ = data[4]<<8;
	wheel_speed->fields.speedZ += data[5];
}

void eslADCS_getRawSunSensor(adcs_raw_sun_t* raw_sun)
{
	unsigned char data[6];
	unsigned char comm = 165;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);
	raw_sun->fields.sun_centroid_x = data[0]<<8;
	raw_sun->fields.sun_centroid_x += data[1];
	raw_sun->fields.sun_centroid_y = data[2]<<8;
	raw_sun->fields.sun_centroid_y += data[3];
	raw_sun->fields.sun_busystatus = data[4];
	raw_sun->fields.sun_result = data[5];
}

void eslADCS_getRawCssMeasurements(adcs_raw_css_t* raw_css)
{
	unsigned char data[6];
	unsigned char comm = 166;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);
	raw_css->fields.css_1 = data[0];
	raw_css->fields.css_2 = data[1];
	raw_css->fields.css_3 = data[2];
	raw_css->fields.css_4 = data[3];
	raw_css->fields.css_5 = data[4];
	raw_css->fields.css_6 = data[5];
}

void eslADCS_getCurrentTime(adcs_unixtime_t* unix_time)
{
	unsigned char data[4];
	unsigned char comm = 2;
	I2C_write(0x12,&comm,1);
	//ADCS_commands(comm, NULL, 0);
	I2C_read(0x12, data, 4);
	unix_time->fields.unix_time_sec = data[0]<<24;
	unix_time->fields.unix_time_sec = data[1]<<16;
	unix_time->fields.unix_time_sec = data[2]<<8;
	unix_time->fields.unix_time_sec += data[3];
}

void eslADCS_getPwrTempTlm(adcs_pwrtemptlm_t* pwrtemp_tlm)
{
	unsigned char comm = 135;
	unsigned char data[18];
	I2C_write(0x12, &comm, 1);
	I2C_read(0x12, data, 18);
	pwrtemp_tlm->fields.csense_3v3curr = data[0] << 8;
	pwrtemp_tlm->fields.csense_3v3curr += data[1];
	pwrtemp_tlm->fields.csense_nadirSRAMcurr = data[2];
	pwrtemp_tlm->fields.csense_sunSRAMcurr = data[3];
	pwrtemp_tlm->fields.arm_cpuTemp = data[4] << 8;
	pwrtemp_tlm->fields.arm_cpuTemp += data[5];
	pwrtemp_tlm->fields.ccontrol_3v3curr = data[6] << 8;
	pwrtemp_tlm->fields.ccontrol_3v3curr += data[7];
	pwrtemp_tlm->fields.ccontrol_5Vcurr = data[8] << 8;
	pwrtemp_tlm->fields.ccontrol_5Vcurr += data[9];
	pwrtemp_tlm->fields.ccontrol_Vbatcurr = data[10] << 8;
	pwrtemp_tlm->fields.ccontrol_Vbatcurr += data[11];
	pwrtemp_tlm->fields.magtorquer_curr = data[12] << 8;
	pwrtemp_tlm->fields.magtorquer_curr += data[13];
	pwrtemp_tlm->fields.momentum_wheelcurr = data[14] << 8;
	pwrtemp_tlm->fields.momentum_wheelcurr += data[15];
	pwrtemp_tlm->fields.ratesensor_temp = data[16];
	pwrtemp_tlm->fields.magnetometer_temp = data[17];

}

void eslADCS_telemetry_Time_Power_temp()
{
	adcs_pwrtemptlm_t pwrtemp_tlm;
	ADCS_telemetry_data telemetry_data;
	eslADCS_getPwrTempTlm(&pwrtemp_tlm);
	telemetry_data.csense_3v3curr = pwrtemp_tlm.fields.ccontrol_3v3curr;
	telemetry_data.csense_nadirSRAMcurr = pwrtemp_tlm.fields.csense_nadirSRAMcurr;
	telemetry_data.csense_sunSRAMcurr = pwrtemp_tlm.fields.csense_sunSRAMcurr;
	telemetry_data.arm_cpuTemp = pwrtemp_tlm.fields.arm_cpuTemp;
	telemetry_data.ccontrol_3v3curr = pwrtemp_tlm.fields.ccontrol_3v3curr;
	telemetry_data.ccontrol_5Vcurr = pwrtemp_tlm.fields.ccontrol_5Vcurr;
	telemetry_data.ccontrol_Vbatcurr = pwrtemp_tlm.fields.ccontrol_Vbatcurr;
	telemetry_data.magtorquer_curr = pwrtemp_tlm.fields.magtorquer_curr;
	telemetry_data.momentum_wheelcurr = pwrtemp_tlm.fields.momentum_wheelcurr;
	telemetry_data.ratesensor_temp = pwrtemp_tlm.fields.ratesensor_temp;
	telemetry_data.magnetometer_temp = pwrtemp_tlm.fields.magnetometer_temp;
}

void eslADCS_Magnetometer_Boom_Deployment_Enabled(Boolean* Magnetometer_Status)
{
	unsigned char comm = 136;
	unsigned char data[48];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(50 / portTICK_RATE_MS);
	I2C_read(0x12, data, 48);
	Magnetometer_Status = data[12];
}


void ADCS_command(unsigned char id, unsigned char* data, unsigned int dat_len)
{
	//unsigned char arr[dat_len + 1];
	unsigned char arr[10]; //change this
	unsigned char ID131 = 131;
	unsigned char Ak[4];

	unsigned int i;
	Boolean Flag = 0;
	arr[0] = id;

	for(i = 1; i < dat_len + 1; i++)
	{
		arr[i] = data[i-1];
	}

	I2C_write(0x12,&arr[0],dat_len+1);

	while(!Flag)
	{
		printf("waiting for love\n");
		I2C_write(0x12,&ID131,1);
		vTaskDelay(60 / portTICK_RATE_MS);
		I2C_read(0x12,Ak,4);
		printf("%x %x %x %x\n",(int)Ak[0],(int)Ak[1],(int)Ak[2],(int)Ak[3]);
		if(Ak[1]==1)
		{
			Flag = 1;
		}
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void eslADCS_getRawMagnetometerMeas(adcs_raw_magmeter_t* raw_mag)
{
	unsigned char data[6];
	unsigned char comm =  167;
	I2C_write(0x12, &comm, 1);
	I2C_read(0x12, data, 6);
	raw_mag->fields.magnetic_x = data[0] << 8;
	raw_mag->fields.magnetic_x += data[1];
	raw_mag->fields.magnetic_y = data[2] << 8;
	raw_mag->fields.magnetic_y += data[3];
	raw_mag->fields.magnetic_z = data[4] << 8;
	raw_mag->fields.magnetic_z += data[5];
}
