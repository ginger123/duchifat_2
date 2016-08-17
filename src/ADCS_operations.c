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
	unsigned char arr[4];
	arr[0] = modesettings.fields.mode;
	arr[1] = modesettings.fields.override;
	arr[2] = modesettings.fields.timeout & 0xff;
	arr[3] = (modesettings.fields.timeout >> 8) & 0xff;
	ADCS_command(18, arr, 4);
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
	arr[5] = (cmd_speed.fields.speedZ >> 8) & 0xff;
	ADCS_command(32, arr, 6);
}

void eslADCS_getEstimatedAngRates(adcs_angrate_t* ang_rates)
{
		unsigned char data[6];
		unsigned char comm= 146;
		//unsigned char comm= 167;
		I2C_write(0x12,&comm,1);
		//ADCS_command(comm, NULL, 0);
		vTaskDelay(5 / portTICK_RATE_MS);
		I2C_read(0x12,data,6);
		//printf("estimated angular rates\n");
		//print_array(data,6);

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
	//printf("get estimated sensor rates\n");
	//print_array(data,6);
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
	//printf("magnetic field vector\n");
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

	unsigned char data[6];
	unsigned char comm = 155;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5);
	I2C_read(0x12,data,6);
	printf("magnetotorquer data from adc: ");
	print_array(data,6);

	mag_cmd->fields.magX = data[0];
	mag_cmd->fields.magX += data[1]<<8;
	mag_cmd->fields.magY = data[2];
	mag_cmd->fields.magY += data[3]<<8;
	mag_cmd->fields.magZ = data[4];
	mag_cmd->fields.magZ += data[5]<<8;
}

void eslADCS_getWheelSpeed(adcs_wheelspeed_t* wheel_speed)
{
	unsigned char data[6];
	unsigned char comm = 154;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5);
	I2C_read(0x12, data, 6);
	wheel_speed->fields.speedX = data[0];
	wheel_speed->fields.speedX += data[1]<<8;
	wheel_speed->fields.speedY = data[2];
	wheel_speed->fields.speedY += data[3]<<8;
	wheel_speed->fields.speedZ = data[4];
	wheel_speed->fields.speedZ += data[5]<<8;
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


/*void eslADCS_getCalSunSensor(adcs_raw_sun_t* raw_sun)
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
}*/



void eslADCS_getRawCssMeasurements(adcs_raw_css_t* raw_css)
{
	unsigned char data[6];
	unsigned char comm = 166;
	I2C_write(0x12,&comm,1);
	vTaskDelay(5);
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
	unix_time->fields.unix_time_sec += data[1]<<16;
	unix_time->fields.unix_time_sec += data[2]<<8;
	unix_time->fields.unix_time_sec += data[3];
}


void eslADCS_getPwrTempTlm(adcs_pwrtemptlm_t* pwrtemp_tlm)
{
	unsigned char comm = 135;
	unsigned char data[18];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, data, 18);

	print_array(data,18);

	pwrtemp_tlm->fields.csense_3v3curr = data[0];
	pwrtemp_tlm->fields.csense_3v3curr += data[1] << 8;
	pwrtemp_tlm->fields.csense_nadirSRAMcurr = data[2];
	pwrtemp_tlm->fields.csense_sunSRAMcurr = data[3];
	pwrtemp_tlm->fields.arm_cpuTemp = data[4] ;
	pwrtemp_tlm->fields.arm_cpuTemp += data[5]<< 8;
	pwrtemp_tlm->fields.ccontrol_3v3curr = data[6];
	pwrtemp_tlm->fields.ccontrol_3v3curr += data[7] << 8;
	pwrtemp_tlm->fields.ccontrol_5Vcurr = data[8];
	pwrtemp_tlm->fields.ccontrol_5Vcurr += data[9] << 8;
	pwrtemp_tlm->fields.ccontrol_Vbatcurr = data[10];
	pwrtemp_tlm->fields.ccontrol_Vbatcurr += data[11] << 8;
	pwrtemp_tlm->fields.magtorquer_curr = data[12];
	pwrtemp_tlm->fields.magtorquer_curr += data[13] << 8;
	pwrtemp_tlm->fields.momentum_wheelcurr = data[14];
	pwrtemp_tlm->fields.momentum_wheelcurr += data[15] << 8;
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
	telemetry_data.stage = adcs_stage;
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
	//print_send_ADCS_telemetry_packet(telemetry_data);
}

void ADCS_get_status(unsigned char *status)
{
	unsigned char comm = 144;
	I2C_write(0x12,	&comm, 1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, status, 6);
	//print_array(status, 6);
}
/*
void eslADCS_Magnetometer_Boom_Deployment_Enabled(Boolean* Magnetometer_Status)
{
	unsigned char comm = 136;
	unsigned char data[48];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(5 / portTICK_RATE_MS);
	I2C_read(0x12, data, 48);
	Magnetometer_Status = data[12];
}*/


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
	vTaskDelay(5);
	I2C_read(0x12, data, 6);
	raw_mag->fields.magnetic_x = data[0];
	raw_mag->fields.magnetic_x += data[1] << 8;
	raw_mag->fields.magnetic_y = data[2];
	raw_mag->fields.magnetic_y += data[3] << 8;
	raw_mag->fields.magnetic_z = data[4];
	raw_mag->fields.magnetic_z += data[5] << 8;
	printf("raw magnetometer values:\n");
	print_array(data,6);
}

void eslADCS_getCurrentPosition(adcs_currstate_t* current_state)
{
	unsigned char data[48];
	unsigned char comm= 136;
	I2C_write(0x12,&comm,1);

	vTaskDelay(5 / portTICK_RATE_MS);

	I2C_read(0x12,data,48);

	current_state->fields.position_x = data[30];
	current_state->fields.position_x += data[31]<<8;
	current_state->fields.position_y = data[32];
	current_state->fields.position_y += data[33]<<8;
	current_state->fields.position_z = data[34];
	current_state->fields.position_z += data[35]<<8;
}

void print_payload_header(adcs_angrate_t ang_rates, adcs_attangles_t att_angles,adcs_currstate_t current_state,unsigned long t)
{

	// print the header
	printf("epoch time: %lu\n",t);
	printf("estimated attitude angles -  roll: %f, pitch: %f, yaw %f [deg]\n",0.01*att_angles.fields.roll,0.01*att_angles.fields.pitch,0.01*att_angles.fields.yaw);
	printf("estimated angular rates - rolldot: %f pitchdot: %f yawdot %f [deg/s]\n",0.01*ang_rates.fields.x_angrate,0.01*ang_rates.fields.y_angrate,0.01*ang_rates.fields.z_angrate);
	printf("position - X: %f, Y: %f, Z: %f\n",0.25*current_state.fields.position_x,0.25*current_state.fields.position_y,0.25*current_state.fields.position_z);
}

void ADCS_payload_Telemetry(ADCS_Payload_Telemetry *Payload_Telemtry)
{
	adcs_angrate_t ang_rates;
	adcs_attangles_t att_angles;
	adcs_currstate_t current_state;
	unsigned long t;

	Time_getUnixEpoch(&t);
	Payload_Telemtry->epoch_time = (int)t-UNIX_EPOCH_TIME_DIFF;
	eslADCS_getEstimatedAttAngles(&att_angles);
	Payload_Telemtry->estimated_attitude_angles[0] = att_angles.fields.roll;
	Payload_Telemtry->estimated_attitude_angles[1] = att_angles.fields.pitch;
	Payload_Telemtry->estimated_attitude_angles[2] = att_angles.fields.yaw;
	eslADCS_getEstimatedAngRates(&ang_rates);
	Payload_Telemtry->estimated_anglar_rates[0] = ang_rates.fields.x_angrate*10;
	Payload_Telemtry->estimated_anglar_rates[1] = ang_rates.fields.y_angrate*10;
	Payload_Telemtry->estimated_anglar_rates[2] = ang_rates.fields.z_angrate*10;
	eslADCS_getCurrentPosition(&current_state);
	Payload_Telemtry->current_Position[0]= current_state.fields.position_x/2;
	Payload_Telemtry->current_Position[1]= current_state.fields.position_y/2;
	Payload_Telemtry->current_Position[2]= current_state.fields.position_z/2;

	print_payload_header(ang_rates,att_angles,current_state,t);
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
	mnlp_header.demo = 0xaaaa;
	//for (i=0;i<174;i++)
	//{
	//	mnlp_pck.mnlp_data[i] = packet[i];
	//}
	ADCS_payload_Telemetry(&mnlp_header);

	printf("full packet header size %d:\n",sizeof(ADCS_Payload_Telemetry));
	print_array((unsigned char *)&mnlp_header,MNLP_HEADER_SIZE);
	print_array(packet,MNLP_DATA_SIZE);
	WritewithEpochtime(sd_file_name, 0, (char *)&mnlp_header, MNLP_HEADER_SIZE);
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
	unsigned char save_com = 100;
	ADCS_Payload_Telemetry Payload_Telemtry;

	//unsigned char temp_tle[] = {0x3F,0xFB,0x58,0xE2,0x19,0x65,0x2B,0xD4	  ,0x3F,0x56,0x34,0xF5,0xAB,0x12,0x67,0xE2  ,0x3F,0xFA,0x66,0x66,0x66,0x66,0x66,0x66 ,0x3F,0xC1,0x37,0x4B,0xC6,0xA7,0xEF,0x9E  ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x3F ,0xB0 ,0xA2 ,0x87 ,0x7E ,0xE4 ,0xE2 ,0x6D ,0x40 ,0x18 ,0x9A ,0x9F ,0xBE ,0x76 ,0xC8 ,0xB4 ,0x41 ,0xD5 ,0xDF ,0x63 ,0x91 ,0x00 ,0x00 ,0x00 ,0xAD ,0x79 ,0x5D ,0x77};
	printf("this is the tle before switch endianess\n");
	print_array(tle,64);
	for(i=0;i<8;i++)//8 parameters to go over
	{
		double_little_endian(tle+8*i);
	}
	printf("this is the tle after switch endianess\n");

	//print_array(tle,64);
	ADCS_command(64,tle,64);
	I2C_write(0x12, &save_com, 1);
	// for testimng
	ADCS_payload_Telemetry(&Payload_Telemtry);
}

void ADCS_set_magnetometer_config(unsigned char mag_config_set[30])
{
	unsigned char set_comm=86,save_com = 100;

	adcs_calibration calibration;

	ADCS_command(set_comm,mag_config_set,30);

	I2C_write(0x12, &save_com, 1);

	eslADCS_getCalibration(&calibration);
	printf("the following are values after settings\n");
	print_calibration( &calibration);
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
	//print_array(data,6);

}

void adcs_reset(unsigned char type)
{
	unsigned char reset_array[2];
	reset_array[0]= 0x01;
	reset_array[1] = type;

	I2C_write(0x12,reset_array,2);
	printf("delay for 15 seconds\n");
	vTaskDelay(15000);
}

void print_estimation_params(unsigned char *ptr)
{
	int   param_int;
	float param_f;

	param_int = ptr[0] + 256*ptr[1]+(256^2)*ptr[2]+(256^3)*ptr[3];
	param_f = (float)param_int;
	printf("system noise %f\n ",param_f);

	ptr+=4;
	param_int = ptr[0] + 256*ptr[1]+(256^2)*ptr[2]+(256^3)*ptr[3];
	param_f = (float)param_int;
	printf("Css measurement noise %f\n ",param_f);

}

void adcs_set_estimation_param(unsigned char mask_sensors)
{
	unsigned char comm = 192;
	unsigned char data[240];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(20 / portTICK_RATE_MS);
	I2C_read(0x12, data, 240);
	unsigned char * pointer_to_first_byte = &data[214];
	printf("parameters before:\n");

	print_estimation_params(pointer_to_first_byte);
	pointer_to_first_byte[24] = mask_sensors;
	//ADCS_command(91,pointer_to_first_byte,26);
	//printf("parameters after:\n");
	//print_estimation_params(pointer_to_first_byte);
}

void eslADCS_getSatelliteVelocityVec(adcs_ecirefvel_t* sat_vel)
{
	unsigned char comm = 148;
	unsigned char data[6];
	I2C_write(0x12, &comm, 1);
	vTaskDelay(5);
	I2C_read(0x12, data, 6);

	sat_vel->fields.x_velocity = data[1] << 8;
	sat_vel->fields.x_velocity += data[0];
	sat_vel->fields.y_velocity = data[3] << 8;
	sat_vel->fields.y_velocity += data[2];
	sat_vel->fields.z_velocity = data[5] << 8;
	sat_vel->fields.z_velocity += data[4];

}

void print_send_ADCS_telemetry_packet(ADCS_telemetry_data telemetry_data)
{
	ccsds_packet ccs_packet;
	int end_offset = 12;

	printf("current cubesence 3.3 %f mA\n",telemetry_data.csense_3v3curr*0.1);
	printf("current Nadir SRAM 3.3 %f mA\n",(float)telemetry_data.csense_nadirSRAMcurr*10);
	printf("current nadir SRAM current %f mA\n",(float)telemetry_data.csense_sunSRAMcurr*10);
	printf(" temp ARM CPU %d C\n",telemetry_data.arm_cpuTemp);
	printf("current cubecontrol 3.3 %f mA\n",telemetry_data.ccontrol_3v3curr*0.48828);
	printf("current cubecontrol 5 %f mA\n",telemetry_data.ccontrol_5Vcurr*0.48828);
	printf("current VBatt 5 %f mA\n",telemetry_data.ccontrol_Vbatcurr*0.48828);
	printf("magnetometer current %f mA\n",telemetry_data.magtorquer_curr*0.1);
	printf("wheel current %f mA\n",telemetry_data.momentum_wheelcurr*0.01);
	printf("temp rate sensor %d C\n",telemetry_data.ratesensor_temp);
	printf("magnetometer %d C\n",telemetry_data.magnetometer_temp);
	printf("general status %x,%x,%x,%x,%x,%x\n",telemetry_data.status[0],telemetry_data.status[1],telemetry_data.status[2],telemetry_data.status[3],telemetry_data.status[4],telemetry_data.status[5]);

	// send packet

	ccs_packet.apid=10;
	ccs_packet.srvc_type = 3;
	ccs_packet.srvc_subtype = 25;
	update_time(ccs_packet.c_time);
	ccs_packet.len = sizeof(ADCS_telemetry_data);
	ccs_packet.data = (unsigned char*)&telemetry_data;

	switch_endian(ccs_packet.data + end_offset, sizeof(ADCS_telemetry_data) - end_offset);
	//send_SCS_pct(ccs_packet);

}

void print_commissioning_packet(ADCS_comissioning_data commissioning_data)
{
    //printf("angular rates raw: %f pitch %f yaw %f deg/s\n",commissioning_data.estimated_anglar_rates[0]*0.01,commissioning_data.estimated_anglar_rates[1]*0.01,commissioning_data.estimated_anglar_rates[2]*0.01);
	printf("stage is %d\n",(int)commissioning_data.stage);
	printf("estimated angular rates: %f pitch %f yaw %f deg/s\n",commissioning_data.estimated_anglar_rates[0]*0.01,commissioning_data.estimated_anglar_rates[1]*0.01,commissioning_data.estimated_anglar_rates[2]*0.01);
    //printf("attitude angles raw: %f pitch %f yaw %f deg\n",commissioning_data.estimated_attitude_angles[0]*0.01,commissioning_data.estimated_attitude_angles[1]*0.01,commissioning_data.estimated_attitude_angles[2]*0.01);
	printf("estimated attitude angles: %f pitch %f yaw %f deg\n",commissioning_data.estimated_attitude_angles[0]*0.01,commissioning_data.estimated_attitude_angles[1]*0.01,commissioning_data.estimated_attitude_angles[2]*0.01);
    printf("Rate sensor rates: X: %f Y: %f Z: %f deg/s\n",commissioning_data.sensor_rates[0]*0.01,commissioning_data.sensor_rates[1]*0.01,commissioning_data.sensor_rates[2]*0.01);

    printf("CSS (Cos sun) 1: %d, 2: %d 3: %d 4: %d 5: %d 6: %d\n",commissioning_data.RAW_CSS[0],commissioning_data.RAW_CSS[1],commissioning_data.RAW_CSS[2],commissioning_data.RAW_CSS[3],commissioning_data.RAW_CSS[4],commissioning_data.RAW_CSS[5]);
    printf("Nadir Azimuth angle: %f, Elevation angle: %f, Busy status: %x, detection result: %x\n",commissioning_data.RAW_nadir_sensors[0]*0.01,commissioning_data.RAW_nadir_sensors[1]*0.01,commissioning_data.RAW_nadir_sensors[2],commissioning_data.RAW_nadir_sensors[3]);
    printf("Sun Azimuth angle: %f, Elevation angle: %f, Busy status: %x, detection result: %x\n",commissioning_data.RAW_sun_sensors[0]*0.01,commissioning_data.RAW_sun_sensors[1]*0.01,commissioning_data.RAW_sun_sensors[2],commissioning_data.RAW_sun_sensors[3]);

    printf("position: Longitude: %f Latitude: %f, Altitude %f deg\n",commissioning_data.sattelite_position[0]*0.01,commissioning_data.sattelite_position[1]*0.01,commissioning_data.sattelite_position[2]*0.02);
    printf("Velocity: X: %f Y: %f, Z %f km/s\n",commissioning_data.sattelite_velocity[0]*0.001,commissioning_data.sattelite_velocity[1]*0.001,commissioning_data.sattelite_velocity[2]*0.001);
    printf("Magnetic field vector - X: %d Y - %d Z - %d nT\n",commissioning_data.magnetic_field_vactor[0]*10,commissioning_data.magnetic_field_vactor[1]*10,commissioning_data.magnetic_field_vactor[2]*10);
    printf("wheel speed X: %d, Y: %d, Z: %d rpm\n",commissioning_data.wheel_speed_estimation[0],commissioning_data.wheel_speed_estimation[1],commissioning_data.wheel_speed_estimation[2]);
    printf("Mtq commands X: %f, Y: %f, Z: %f\n",commissioning_data.Magnetorquer_commands[0]*0.001,commissioning_data.Magnetorquer_commands[1]*0.001,commissioning_data.Magnetorquer_commands[2]*0.001);
}

void test_ADCS_packet()
{
	ADCS_telemetry_data telemetry_data;
	telemetry_data.sid = 159;
	telemetry_data.ratesensor_temp = 1;
	telemetry_data.magnetometer_temp = 2;
	telemetry_data.csense_nadirSRAMcurr = 3;
	telemetry_data.csense_sunSRAMcurr = 4;
	telemetry_data.stage = 10;
	telemetry_data.status[0] = 5;
	telemetry_data.status[1] = 6;
	telemetry_data.status[2] = 7;
	telemetry_data.status[3] = 8;
	telemetry_data.status[4] = 9;
	telemetry_data.status[5] = 10;
	telemetry_data.csense_3v3curr = 11;
	telemetry_data.arm_cpuTemp = 12;
	telemetry_data.ccontrol_3v3curr = 13;
	telemetry_data.ccontrol_5Vcurr = 14;
	telemetry_data.ccontrol_Vbatcurr = 15;
	telemetry_data.magtorquer_curr = 16;
	telemetry_data.momentum_wheelcurr = 17;
	print_send_ADCS_telemetry_packet(telemetry_data);
}
void test_commissioning_packet()
{
	ADCS_comissioning_data adc_dat;
	adc_dat.sid= 201;
	adc_dat.stage= 10;
	adc_dat.estimated_anglar_rates[0]=1;
	adc_dat.estimated_anglar_rates[1]=2;
	adc_dat.estimated_anglar_rates[2]=3;
	adc_dat.magnetic_field_vactor[0]=4;
	adc_dat.magnetic_field_vactor[1]=5;
	adc_dat.magnetic_field_vactor[2]=6;
	adc_dat.sensor_rates[0]=7;
	adc_dat.sensor_rates[1]=8;
	adc_dat.sensor_rates[2]=9;
	adc_dat.RAW_Magnetometer[0]=10;
	adc_dat.RAW_Magnetometer[1]=11;
	adc_dat.RAW_Magnetometer[2]=12;
	adc_dat.Magnetorquer_commands[0]=13;
	adc_dat.Magnetorquer_commands[1]=14;
	adc_dat.Magnetorquer_commands[2]=15;
	adc_dat.estimated_attitude_angles[0]=16;
	adc_dat.estimated_attitude_angles[1]=17;
	adc_dat.estimated_attitude_angles[2]=18;
	adc_dat.sattelite_position[0]=19;
	adc_dat.sattelite_position[1]=20;
	adc_dat.sattelite_position[2]=21;
	adc_dat.sattelite_velocity[0]=22;
	adc_dat.sattelite_velocity[1]=23;
	adc_dat.sattelite_velocity[2]=24;
	adc_dat.RAW_nadir_sensors[0]=25;
	adc_dat.RAW_nadir_sensors[1]=26;
	adc_dat.RAW_nadir_sensors[2]=27;
	adc_dat.RAW_nadir_sensors[3]=28;
	adc_dat.RAW_sun_sensors[0]=29;
	adc_dat.RAW_sun_sensors[1]=30;
	adc_dat.RAW_sun_sensors[2]=31;
	adc_dat.RAW_sun_sensors[3]=32;
	adc_dat.wheel_speed_estimation[0]=33;
	adc_dat.wheel_speed_estimation[1]=34;
	adc_dat.wheel_speed_estimation[2]=35;
	adc_dat.wheel_speed_command[0]=36;
	adc_dat.wheel_speed_command[1]=37;
	adc_dat.wheel_speed_command[2]=38;
	adc_dat.RAW_CSS[0]=39;
	adc_dat.RAW_CSS[1]=40;
	adc_dat.RAW_CSS[2]=41;
	adc_dat.RAW_CSS[3]=42;
	adc_dat.RAW_CSS[4]=43;
	adc_dat.RAW_CSS[5]=44;
	print_commissioning_packet(adc_dat);
	switch_endian(((unsigned char*)&adc_dat)+8,sizeof(ADCS_comissioning_data)-8);
	ccsds_packet ccd;
	ccd.apid=10;
	ccd.srvc_type=3;
	ccd.srvc_subtype=25;
	ccd.data=(unsigned char*) &adc_dat;
	ccd.len = sizeof(ADCS_comissioning_data);
	update_time(ccd.c_time);
	send_SCS_pct(ccd);
}
