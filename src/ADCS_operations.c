#include "main.h"
#include "ADCS_operations.h"
#include "ADCS_Thread.h"

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
	arr[0] = (unsigned char)current_status;
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
		vTaskDelay(5 / portTICK_RATE_MS);
		I2C_read(0x12,data,6);
		printf("estimated angular rates\n");
		print_array(data,6);

		ang_rates->fields.x_angrate = data[1]<<8;
		ang_rates->fields.x_angrate += data[0];
		ang_rates->fields.y_angrate = data[3]<<8;
		ang_rates->fields.y_angrate += data[2];
		ang_rates->fields.z_angrate = data[5]<<8;
		ang_rates->fields.z_angrate += data[4];
}

void eslADCS_getSensorRates(adcs_angrate_t* sen_rates)
{
	unsigned char data[6];
	unsigned char comm = 153;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12,data,6);
	printf("get estimated sensor rates\n");
	print_array(data,6);
	sen_rates->fields.x_angrate = data[1]<<8;
	sen_rates->fields.x_angrate += data[0];
	sen_rates->fields.y_angrate = data[3]<<8;
	sen_rates->fields.y_angrate += data[2];
	sen_rates->fields.z_angrate = data[5]<<8;
	sen_rates->fields.z_angrate += data[4];
}

void eslADCS_getEstimatedAttAngles(adcs_attangles_t *att_angles)
{
	unsigned char data[6];
	unsigned char comm = 145;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12,data,6);

	att_angles->fields.roll = data[0];
	att_angles->fields.roll += data[1] << 8;
	att_angles->fields.pitch = data[2] ;
	att_angles->fields.pitch += data[3]<< 8;
	att_angles->fields.yaw = data[4] ;
	att_angles->fields.yaw += data[5]<< 8;

}

void eslADCS_getMagneticFieldVec(adcs_magfieldvec_t* mag_field)
{
	unsigned char data[6];
	unsigned char comm = 149;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12,data,6);
	printf("magnetic field vector\n");
	print_array(data,6);

	mag_field->fields.x_magfield = data[1]<<8;
	mag_field->fields.x_magfield += data[0];
	mag_field->fields.y_magfield = data[3]<<8;
	mag_field->fields.y_magfield += data[2];
	mag_field->fields.z_magfield = data[5]<<8;
	mag_field->fields.z_magfield += data[4];
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

void eslADCS_getRawNadirSensor(adcs_raw_nadir_t *raw_nadir)
{
	unsigned char data[6];
	unsigned char comm = 164;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5 / portTICK_RATE_MS);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);
	printf("Raw Nadir sensor measurements\n");
	print_array(data,6);
	raw_nadir->fields.nadir_centroid_x = data[1]<<8;
	raw_nadir->fields.nadir_centroid_x += data[0];
	raw_nadir->fields.nadir_centroid_y = data[3]<<8;
	raw_nadir->fields.nadir_centroid_y += data[2];
	raw_nadir->fields.nadir_busystatus = data[4];
	raw_nadir->fields.nadir_result = data[5];
}

void eslADCS_getRawSunSensor(adcs_raw_sun_t* raw_sun)
{
	unsigned char data[6];
	unsigned char comm = 165;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5 / portTICK_RATE_MS);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);
	printf("Raw sun sensor measurements\n");
	print_array(data,6);
	raw_sun->fields.sun_centroid_x = data[1]<<8;
	raw_sun->fields.sun_centroid_x += data[0];
	raw_sun->fields.sun_centroid_y = data[3]<<8;
	raw_sun->fields.sun_centroid_y += data[2];
	raw_sun->fields.sun_busystatus = data[4];
	raw_sun->fields.sun_result = data[5];
}

void eslADCS_getCalNadirSensor()
{
	unsigned char data[6];
	unsigned char comm = 152;
	short nadir_vec[3];
	I2C_write(0x12,&comm,1);
	vTaskDelay(5 / portTICK_RATE_MS);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);

	printf("Raw Madir sensor measurements\n");
	print_array(data,6);

	nadir_vec[0] =data[1]<<8;
	nadir_vec[0] +=data[0];
	nadir_vec[1] =data[3]<<8;
	nadir_vec[1] +=data[2];
	nadir_vec[2] =data[5]<<8;
	nadir_vec[2] +=data[4];

	printf("X nadir vector is %f\n",(float)nadir_vec[0]/32768);
	printf("Y nadir vector is %f\n",(float)nadir_vec[1]/32768);
	printf("Z nadir vector is %f\n",(float)nadir_vec[2]/32768);

}

void eslADCS_getCalSunSensor(adcs_raw_sun_t* raw_sun)
{
	unsigned char data[6];
		unsigned char comm = 151;
		short sun_vec[3];
		I2C_write(0x12,&comm,1);
		vTaskDelay(5 / portTICK_RATE_MS);
		//ADCS_command(comm, NULL, 0);
		I2C_read(0x12, data, 6);

		printf("Raw sun sensor measurements\n");
		print_array(data,6);

		sun_vec[0] =data[1]<<8;
		sun_vec[0] +=data[0];
		sun_vec[1] =data[3]<<8;
		sun_vec[1] +=data[2];
		sun_vec[2] =data[5]<<8;
		sun_vec[2] +=data[4];

		printf("X nadir vector is %f\n",(float)sun_vec[0]/32768);
		printf("Y nadir vector is %f\n",(float)sun_vec[1]/32768);
		printf("Z nadir vector is %f\n",(float)sun_vec[2]/32768);
}



void eslADCS_getRawCssMeasurements(adcs_raw_css_t* raw_css)
{
	unsigned char data[6];
	unsigned char comm = 166;
	I2C_write(0x12,&comm,1);
	//ADCS_command(comm, NULL, 0);
	I2C_read(0x12, data, 6);
	printf("CSS measurements\n");
	print_array(data,6);

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
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, data, 18);

	//print_array(data,18);

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
	ADCS_telemetry_data telemetry_data;
	adcs_pwrtemptlm_t pwrtemp_tlm;

	eslADCS_getPwrTempTlm(&pwrtemp_tlm);
	ADCS_get_status(telemetry_data.status);
	telemetry_data.sid=159;
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
	//printf("printing adcs telemetry\n");
	//print_array((unsigned char*)&telemetry_data,sizeof(ADCS_telemetry_data));
	//printf("3v3 curr %d\n",telemetry_data.csense_3v3curr);
	//printf("arm cputemp %d\n",telemetry_data.arm_cpuTemp);
	//printf("telemtry 3v3curr %d\n",telemetry_data.ccontrol_3v3curr);
	WritewithEpochtime("adcs_tlm_file",0,(char *) &telemetry_data, sizeof(ADCS_telemetry_data));
}

void ADCS_get_status(unsigned char *status)
{
	unsigned char comm = 144;
	I2C_write(0x12,	&comm, 1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, status, 6);
	//print_array(status, 6);
}

void eslADCS_Magnetometer_Boom_Deployment_Enabled(Boolean* Magnetometer_Status)
{
	unsigned char comm = 136;
	unsigned char data[48];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, data, 48);
	Magnetometer_Status = data[12];
}


void ADCS_command(unsigned char id, unsigned char* data, unsigned int dat_len)
{
	//unsigned char arr[dat_len + 1];
	unsigned char arr[80];
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
		if(Ak[1])
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

void eslADCS_getCurrentPosition(adcs_currstate_t* current_state)
{
	unsigned char data[48];
	unsigned char comm= 136;
	I2C_write(0x12,&comm,1);

	vTaskDelay(50 / portTICK_RATE_MS);

	I2C_read(0x12,data,48);

	current_state->fields.position_x = data[40]<<8;
	current_state->fields.position_x += data[41];
	current_state->fields.position_y = data[42]<<8;
	current_state->fields.position_y += data[43];
	current_state->fields.position_z = data[44]<<8;
	current_state->fields.position_z += data[45];
}

void ADCS_payload_Telemetry(ADCS_Payload_Telemetry *Payload_Telemtry)
{
	adcs_angrate_t ang_rates;
	adcs_attangles_t att_angles;
	adcs_currstate_t current_state;

	eslADCS_getEstimatedAttAngles(&att_angles);
	Payload_Telemtry->estimated_attitude_angles[0] = att_angles.fields.roll;
	Payload_Telemtry->estimated_attitude_angles[1] = att_angles.fields.pitch;
	Payload_Telemtry->estimated_attitude_angles[2] = att_angles.fields.yaw;
	eslADCS_getEstimatedAngRates(&ang_rates);
	Payload_Telemtry->estimated_anglar_rates[0] = ang_rates.fields.x_angrate;
	Payload_Telemtry->estimated_anglar_rates[1] = ang_rates.fields.y_angrate;
	Payload_Telemtry->estimated_anglar_rates[2] = ang_rates.fields.z_angrate;
	eslADCS_getCurrentPosition(&current_state);
	Payload_Telemtry->current_Position[0]= current_state.fields.position_x;
	Payload_Telemtry->current_Position[1]= current_state.fields.position_y;
	Payload_Telemtry->current_Position[2]= current_state.fields.position_z;
}


void eslADCS_getCalibration(adcs_calibration *calibration)
{
	unsigned char comm = 192;
	unsigned char data[240];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(2000 / portTICK_RATE_MS);
	I2C_read(0x12, data, 240);
	print_array(data,240);
	calibration->mtq[0] = data[0];
	calibration->mtq[1] = data[1];
	calibration->mtq[2] = data[2];
	calibration->mtq_max_mnt[0] = data[4]<<8;
	calibration->mtq_max_mnt[0] += data[3];
	calibration->mtq_max_mnt[1] = data[6]<<8;
	calibration->mtq_max_mnt[1] += data[5];
	calibration->mtq_max_mnt[2] = data[8]<<8;
	calibration->mtq_max_mnt[2] += data[7];
	calibration->wheel_mnt_ang[0] = data[14]<<8;
	calibration->wheel_mnt_ang[0] += data[13];
	calibration->wheel_mnt_ang[1] = data[16]<<8;
	calibration->wheel_mnt_ang[1] += data[15];
	calibration->mtq_mnt_ang[0] = data[115]<<8;
	calibration->mtq_mnt_ang[0] += data[114];
	calibration->mtq_mnt_ang[1] = data[117]<<8;
	calibration->mtq_mnt_ang[1] += data[116];
	calibration->mtq_mnt_ang[2] = data[119]<<8;
	calibration->mtq_mnt_ang[2] += data[118];
	calibration->mtq_chanel[0] = data[121]<<8;
	calibration->mtq_chanel[0] += data[120];
	calibration->mtq_chanel[1] = data[123]<<8;
	calibration->mtq_chanel[1] += data[122];
	calibration->mtq_chanel[2] = data[125]<<8;
	calibration->mtq_chanel[2] += data[124];
	calibration->mtq_sens_matrix[0] = data[127]<<8;
	calibration->mtq_sens_matrix[0] += data[126];
	calibration->mtq_sens_matrix[1] = data[129]<<8;
	calibration->mtq_sens_matrix[1] += data[128];
	calibration->mtq_sens_matrix[2] = data[131]<<8;
	calibration->mtq_sens_matrix[2] += data[130];
	calibration->Nadir_sensor_mnt_ang[0] = data[58]<<8;
	calibration->Nadir_sensor_mnt_ang[0] += data[57];
	calibration->Nadir_sensor_mnt_ang[1] = data[60]<<8;
	calibration->Nadir_sensor_mnt_ang[1] += data[59];
	calibration->Nadir_sensor_mnt_ang[2] = data[62]<<8;
	calibration->Nadir_sensor_mnt_ang[2] += data[60];
	calibration->sun_sensor_mnt_ang[0] = data[41]<<8;
	calibration->sun_sensor_mnt_ang[0] += data[40];
	calibration->sun_sensor_mnt_ang[1] = data[43]<<8;
	calibration->sun_sensor_mnt_ang[1] += data[42];
	calibration->sun_sensor_mnt_ang[2] = data[45]<<8;
	calibration->sun_sensor_mnt_ang[2] += data[44];
	calibration->css[0] = data[27];
	calibration->css[1] = data[26];
	calibration->css[2] = data[29];
	calibration->css[3] = data[28];
	calibration->css[4] = data[31];
	calibration->css[5] = data[30];
	calibration->Rate_sensor_mnt_ang[0] = data[147]<<8;
	calibration->Rate_sensor_mnt_ang[0] += data[146];
	calibration->Rate_sensor_mnt_ang[1] = data[149]<<8;
	calibration->Rate_sensor_mnt_ang[1] += data[148];
}

void print_calibration(adcs_calibration *calibration)
{

	printf("Magnetorquer 1 %x\n",(int)calibration->mtq[0]);
	printf("Magnetorquer 2 %x\n",(int)calibration->mtq[1]);
	printf("Magnetorquer 3 %x\n",(int)calibration->mtq[2]);

	printf("X Magnetorquer Max Moment %f\n",0.0001*calibration->mtq_max_mnt[0]);
	printf("y Magnetorquer Max Moment %f\n",0.0001*calibration->mtq_max_mnt[1]);
	printf("z Magnetorquer Max Moment %f\n",0.0001*calibration->mtq_max_mnt[2]);

	printf("Wheel mount angle 1 %f\n",0.01*calibration->wheel_mnt_ang[0]);
	printf("Wheel mount angle 2 %f\n",0.01*calibration->wheel_mnt_ang[1]);

	printf("Magnetometer mount angel 1 %f\n",0.01*calibration->mtq_mnt_ang[0]);
	printf("Magnetometer mount angel 2 %f\n",0.01*calibration->mtq_mnt_ang[1]);
	printf("Magnetometer mount angel 3 %f\n",0.01*calibration->mtq_mnt_ang[2]);

	printf("Magnetometer chanel 1 offset %x\n",calibration->mtq_chanel[0]);
	printf("Magnetometer chanel 2 offset %x\n",calibration->mtq_chanel[1]);
	printf("Magnetometer chanel 3 offset %x\n",calibration->mtq_chanel[2]);

	printf("Magnetometer sensitivity matrix s11  %f\n",0.001*calibration->mtq_sens_matrix[0]);
	printf("Magnetometer sensitivity matrix s12  %f\n",0.001*calibration->mtq_sens_matrix[1]);
	printf("Magnetometer sensitivity matrix s13  %f\n",0.001*calibration->mtq_sens_matrix[2]);

	printf("Nadir sensor mount angle 1 %f\n",0.01*calibration->Nadir_sensor_mnt_ang[0]);
	printf("Nadir sensor mount angle 2 %f\n",0.01*calibration->Nadir_sensor_mnt_ang[1]);
	printf("Nadir sensor mount angle 3 %f\n",0.01*calibration->Nadir_sensor_mnt_ang[2]);

	printf("sun sensor mount angle 1 %f\n",0.01*calibration->sun_sensor_mnt_ang[0]);
	printf("sun sensor mount angle 2 %f\n",0.01*calibration->sun_sensor_mnt_ang[1]);
	printf("sun sensor mount angle 3 %f\n",0.01*calibration->sun_sensor_mnt_ang[2]);

	printf("css 1 %x\n",calibration->css[0]);
	printf("css 2 %x\n",calibration->css[1]);
	printf("css 3 %x\n",calibration->css[2]);
	printf("css 4 %x\n",calibration->css[3]);
	printf("css 5 %x\n",calibration->css[4]);
	printf("css 6 %x\n",calibration->css[5]);

	printf("Y-Rate sensor mount angle 1 %f\n",0.01*calibration->Rate_sensor_mnt_ang[0]);
	printf("Y-Rate sensor mount angle 2 %f\n",0.01*calibration->Rate_sensor_mnt_ang[1]);
}



void Build_PayloadPacket(unsigned char *packet)
{
	char sd_file_name[] = {"mnlp"};

	ADCS_Payload_Telemetry mnlp_header;
	//for (i=0;i<174;i++)
	//{
	//	mnlp_pck.mnlp_data[i] = packet[i];
	//}
	ADCS_payload_Telemetry(&mnlp_header);


	WritewithEpochtime(sd_file_name, 0, (char *)&mnlp_header, sizeof(ADCS_Payload_Telemetry));
	FileWrite(sd_file_name, 0, ( char *)packet, MNLP_DATA_SIZE);

}

void idtlm()
{
	unsigned char data[8];
		unsigned char comm= 128;
		I2C_write(0x12,&comm,1);

		vTaskDelay(50 / portTICK_RATE_MS);

		I2C_read(0x12,data,8);

		//printf("ADCS TLM1 28\n");
		//print_array(data,8);
}

void ADCS_update_unix_time(unsigned long t)
{
	unsigned char data[6]={t&0xff,(t>>8)&0xff,(t>>16)&0xff,(t>>24)&0xff,0,0};
	ADCS_command(2,data,6);
}

void ADCS_update_tle(unsigned char* tle)
{
	int i;

	unsigned char temp_tle[] = {0x3F,0xFB,0x58,0xE2,0x19,0x65,0x2B,0xD4	  ,0x3F,0x56,0x34,0xF5,0xAB,0x12,0x67,0xE2  ,0x3F,0xFA,0x66,0x66,0x66,0x66,0x66,0x66 ,0x3F,0xC1,0x37,0x4B,0xC6,0xA7,0xEF,0x9E  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x3F ,0xB0 ,0xA2 ,0x87 ,0x7E ,0xE4 ,0xE2 ,0x6D ,0x40 ,0x18 ,0x9A ,0x9F ,0xBE ,0x76 ,0xC8 ,0xB4 ,0x41 ,0xD5 ,0xDF ,0x63 ,0x91 ,0x00 ,0x00 ,0x00 ,0xAD ,0x79 ,0x5D ,0x77};

	for(i=0;i<8;i++)//8 parameters to go over
	{
		double_little_endian(temp_tle+8*i);
	}
	printf("this is the tle that we are sending to adcs\n");
	print_array(temp_tle,64);
	ADCS_command(64,temp_tle,64);
}

void ADCS_set_magnetometer_config()
{
	unsigned char get_comm=192,set_comm=86,save_com = 100;
	unsigned char get_data[240],set_data[30];
	int i;
	adcs_calibration calibration;


	eslADCS_getCalibration(&calibration);
	printf("the following are values before settings\n");
	print_calibration( &calibration);

	I2C_write(0x12, &get_comm, 1);
	vTaskDelay(500 / portTICK_RATE_MS);
	I2C_read(0x12, get_data, 240);

	// set the offset values and sensiticity matrix
	for (i=0;i<24;i++)
	{
		set_data[6+i] = get_data[120+i];
	}

	// magnetometer angles
	set_data[0] = 0;
	set_data[1] = 0;
	set_data[2] = 0x28;
	set_data[3] = 0x23;
	set_data[4] = 0;
	set_data[5] = 0;
	// set the
	printf("set new data\n");
	ADCS_command(set_comm,set_data,30);
	printf("the following are values after settings\n");
	eslADCS_getCalibration(&calibration);
	print_calibration( &calibration);
	I2C_write(0x12, &save_com, 1);
}

void get_sat_llh_pos(adcs_refllhcoord_t *llh_in)
{
	unsigned char comm = 147;
	unsigned char data[6];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(5);
	I2C_read(0x12, data, 6);
	llh_in->fields.latitude = data[1] << 8;
	llh_in->fields.latitude += data[0];
	llh_in->fields.longitud = data[3] << 8;
	llh_in->fields.longitud += data[2];
	llh_in->fields.altitude = data[5] << 8;
	llh_in->fields.altitude += data[4];
	print_array(data,6);
	printf("latitude is: %f\n", (float)llh_in->fields.latitude * 0.01);
	printf("longitud is: %f\n", (float)llh_in->fields.longitud * 0.01);
	printf("altitude is: %f\n",  (float)llh_in->fields.altitude * 0.02);
}

adcs_reset(unsigned char type)
{
	unsigned char reset_array[2];
	reset_array[0]= 0x01;
	reset_array[1] = type;

	I2C_write(0x12,&reset_array,2);
	printf("delay for 15 seconds\n");
	vTaskDelay(15000);

}
adcs_set_estimation_param()
{
}
