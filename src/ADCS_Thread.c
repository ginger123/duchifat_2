/*
 * ADCS_Thread.c
 *
 *  Created on: 18 באפר 2016
 *      Author: USER1 check git
 */


#include "ADCS_operations.h"


#ifndef NULL
#define NULL ( (void *) 0)
#endif

#ifndef FALSE
#define FALSE  0
#endif


int adcs_stage;
void adc_stages(int stage)
{
	switch (stage)
	{
		case 1:
			printf("%d\n", 1);

			ADC_Stage_1();
			break;
		case 2:
			printf("%d\n", 2);

			ADC_Stage_2();
			break;
		case 3:
			printf("%d\n", 3);

			ADC_Stage_3();
			break;
		case 4:
			printf("%d\n", 4);
			adcs_stage = 4;
			ADC_Stage_4();
			break;
		case 5:
			printf("%d\n", 5);
			adcs_stage = 5;
			ADC_Stage_5();
			break;
		case 6:
			printf("%d\n", 6);
			adcs_stage = 6;
			ADC_Stage_6();
			break;
		case 7:
			printf("%d\n", 7);
			adcs_stage = 7;
			ADC_Stage_7();
			break;

		default:
		break;
	}
}


void ADC_Stage_1()
{

	adcs_stage = 1;

	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on; //motor power on(1)
	Device_ctrl.fields.motor_cubecontrol = selection_auto; //all others auto
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_angrate_t Ang_rates;

	adcs_angrate_t Sen_rates;
	adcs_magfieldvec_t Mag_field;
	ADCS_comissioning_data commisioning_data;

	eslADCS_setStateADCS(state_enabled); //run the ADCS
	eslADCS_setEstimationMode(est_magnetometer_rate); //set the estimation mode
	eslADCS_setPwrCtrlDevice(Device_ctrl); //power on the motor

	while (adcs_stage == 1)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; // getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getMagneticFieldVec(&Mag_field); //filling the Mag_field with data
		commisioning_data.magnetic_field_vactor[0] = Mag_field.fields.x_magfield;
		commisioning_data.magnetic_field_vactor[1] = Mag_field.fields.y_magfield;//getting the data to the commisioning_data struct
		commisioning_data.magnetic_field_vactor[2] = Mag_field.fields.z_magfield;

		printf(" And Rates x: %d\n",commisioning_data.estimated_anglar_rates[0]);
		printf(" And Rates y: %d\n",commisioning_data.estimated_anglar_rates[1]);
		printf(" And Rates z: %d\n",commisioning_data.estimated_anglar_rates[2]);
				//Ang Rates

				//Sen Rates
		printf(" Sen Rates x: %d\n",commisioning_data.sensor_rates[0]);
		printf(" Sen Rates y: %d\n",commisioning_data.sensor_rates[1]);
		printf(" Sen Rates z: %d\n",commisioning_data.sensor_rates[2]);
			//Sen Rates

				//Mag Field Vector
		printf(" Mag Field Vector x: %d\n",commisioning_data.magnetic_field_vactor[0]);
		printf(" Mag Field Vector y: %d\n",commisioning_data.magnetic_field_vactor[1]);
		printf(" Mag Field Vector z: %d\n",commisioning_data.magnetic_field_vactor[2]);
				//Mag Field Vector
		WritewithEpochtime("adcs_file",0, (char *) &commisioning_data, sizeof(ADCS_comissioning_data));
		printf("delay for 10 seconds\n");
		vTaskDelay(10000 / portTICK_RATE_MS);
	}
}

void ADC_Stage_2()
{

	adcs_ctrlmodeset_t modesetting;
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on; //motor power on(1)
	Device_ctrl.fields.motor_cubecontrol = selection_auto; //all others auto
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_angrate_t Ang_rates;
	adcs_angrate_t Sen_rates;
	adcs_magfieldvec_t Mag_field;
	ADCS_comissioning_data commisioning_data;
	adcs_magnetorq_t mag_cmd;
	eslADCS_setStateADCS(state_enabled); //run the ADCS

	eslADCS_setEstimationMode(est_magnetometer_rate); //set the estimation mode

	printf("60 Seconds delay\n");
	vTaskDelay(60000 / portTICK_RATE_MS); //delay of 1min

	modesetting.fields.mode =  ctrl_mode_detumbling; //enter to detumbling mode
	modesetting.fields.timeout = 0;
	eslADCS_setAttitudeCtrlMode(modesetting);
	eslADCS_setPwrCtrlDevice(Device_ctrl); //turn on motor

	while(adcs_stage == 2)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getMagneticFieldVec(&Mag_field); //filling the Mag_field with data
		commisioning_data.magnetic_field_vactor[0] = Mag_field.fields.x_magfield; //getting the data to the commisioning_data struct
		commisioning_data.magnetic_field_vactor[1] = Mag_field.fields.y_magfield;
		commisioning_data.magnetic_field_vactor[2] = Mag_field.fields.z_magfield;
		eslADCS_getMagnetorquerCmd(&mag_cmd); //filling the mag_cmd with data
		commisioning_data.Magnetorquer_commands[0] = mag_cmd.fields.magX;  //getting the data to the commisioning_data struct
		commisioning_data.Magnetorquer_commands[1] = mag_cmd.fields.magY;
		commisioning_data.Magnetorquer_commands[2] = mag_cmd.fields.magZ;
		//FileWrite("adcs_file",0,(char *) &commisioning_data, 0, sizeof(ADCS_comissioning_data),0);
		vTaskDelay(10000 / portTICK_RATE_MS); //delay 10s
		WritewithEpochtime("adcs_file",0, (char *) &commisioning_data, sizeof(ADCS_comissioning_data));

	}
}


void ADC_Stage_3()
{
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on; //motor power on(1)
	Device_ctrl.fields.motor_cubecontrol = selection_auto; //all others auto
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_ctrlmodeset_t modesetting;
	adcs_angrate_t Ang_rates;
	adcs_angrate_t Sen_rates;
	ADCS_comissioning_data commisioning_data;
	adcs_raw_magmeter_t raw_mag;
	Boolean Magnetometer_Status;

	modesetting.fields.mode = ctrl_mode_none; //setting mode to none(0)
	eslADCS_setStateADCS(state_enabled); ////run the ADCS
	eslADCS_setEstimationMode(est_magnetometer_rate); //set the estimation mode
	eslADCS_setAttitudeCtrlMode(modesetting);
	eslADCS_setPwrCtrlDevice(Device_ctrl); //turn on motor power
	eslADCS_deployMagnetometer(2); //first attempt entering what the value of the function to boom deploy
	eslADCS_Magnetometer_Boom_Deployment_Enabled(&Magnetometer_Status);

	if (Magnetometer_Status == FALSE)
	{
		eslADCS_deployMagnetometer(5);
		eslADCS_Magnetometer_Boom_Deployment_Enabled(&Magnetometer_Status);

		if(Magnetometer_Status == FALSE)
		{
			eslADCS_deployMagnetometer(10);
			eslADCS_Magnetometer_Boom_Deployment_Enabled(&Magnetometer_Status);
		}
	}

	 /*
	  * set magnetometer configuration is not done!!!!!
	  * adcsconf_magmeter_t conf_magmeter;
	  * eslADCS_setConfMagMeter
	*/

	eslADCS_setConfSave(0);

	while (adcs_stage == 3)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getRawMagnetometerMeas(&raw_mag); //filling the raw_mag with data
		commisioning_data.RAW_Magnetometer[0] = raw_mag.fields.magnetic_x; //getting the data to the commisioning_data struct
		commisioning_data.RAW_Magnetometer[1] = raw_mag.fields.magnetic_y;
		commisioning_data.RAW_Magnetometer[2] = raw_mag.fields.magnetic_z;
		vTaskDelay(1000 / portTICK_RATE_MS); //dealy of 1s
		WritewithEpochtime("adcs_file",0, (char *) &commisioning_data, sizeof(ADCS_comissioning_data));
	}
}

void ADC_Stage_4()
{
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on;
	Device_ctrl.fields.motor_cubecontrol = selection_auto;
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_ctrlmodeset_t modesetting;
	ADCS_comissioning_data commisioning_data;
	adcs_angrate_t Ang_rates;
	adcs_angrate_t Sen_rates;
	adcs_raw_magmeter_t raw_mag;
	modesetting.fields.mode = ctrl_mode_detumbling;
	eslADCS_setStateADCS(state_enabled);
	eslADCS_setEstimationMode(est_magnetometer_rate);
	eslADCS_setAttitudeCtrlMode(modesetting);
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	vTaskDelay(5400000 / portTICK_RATE_MS); //dealy of 1 orbit

	 /*
		  * set magnetometer configuration is not done!!!!!
		  * adcsconf_magmeter_t conf_magmeter;
		  * eslADCS_setConfMagMeter
	*/

	eslADCS_setConfSave();

	while (adcs_stage == 4)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		//eslADCS_getRawMagnetometerMeas(&raw_mag); //filling the raw_mag with data
		commisioning_data.RAW_Magnetometer[0] = raw_mag.fields.magnetic_x; //getting the data to the commisioning_data struct
		commisioning_data.RAW_Magnetometer[1] = raw_mag.fields.magnetic_y;
		commisioning_data.RAW_Magnetometer[2] = raw_mag.fields.magnetic_z;
		vTaskDelay(10000 / portTICK_RATE_MS); //delay of 10s
		WritewithEpochtime("adcs_file",0,(char *) &commisioning_data, sizeof(ADCS_comissioning_data));
	}
}

void ADC_Stage_5()
{
	unsigned long epochTime;
	adcs_unixtime_t unix_time;
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on;
	Device_ctrl.fields.motor_cubecontrol = selection_auto;
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	//adcs_orbitparam_t orbit_param;
	adcs_ctrlmodeset_t modesetting;
	adcs_estmode_t mode;
	adcs_attangles_t att_angles;
	adcs_refllhcoord_t sat_pos;
	adcs_angrate_t Ang_rates;
	adcs_angrate_t Sen_rates;
	adcs_magfieldvec_t Mag_field;
	adcs_ecirefvel_t sat_vel;
	ADCS_comissioning_data commisioning_data;
	mode = est_magnetometer_ratewithpitch;
	modesetting.fields.mode = ctrl_mode_detumbling;
	unix_time.fields.unix_time_sec = Time_getUnixEpoch(&epochTime);
	eslADCS_setStateADCS(state_enabled);
	eslADCS_getCurrentTime(&unix_time);
	//eslADCS_setOrbitParam() need to complete!!!
	eslADCS_setEstimationMode(mode);
	eslADCS_setAttitudeCtrlMode(modesetting);
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	while (adcs_stage == 5)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		//eslADCS_getEstimatedAttAngles(&att_angles);
		commisioning_data.estimated_attitude_angles[0] = att_angles.fields.pitch;
		commisioning_data.estimated_attitude_angles[1] = att_angles.fields.roll;
		commisioning_data.estimated_attitude_angles[2] = att_angles.fields.yaw;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getMagneticFieldVec(&Mag_field); //filling the Mag_field with data
		commisioning_data.magnetic_field_vactor[0] = Mag_field.fields.x_magfield; //getting the data to the commisioning_data struct
		commisioning_data.magnetic_field_vactor[1] = Mag_field.fields.y_magfield;
		commisioning_data.magnetic_field_vactor[2] = Mag_field.fields.z_magfield;
		//eslADCS_getSatellitePositionVec(&sat_pos); //filling the sat_pos with data
		commisioning_data.sattelite_position[0] = sat_pos.fields.longitud; //getting the data to the commisioning_data struct
		commisioning_data.sattelite_position[1] = sat_pos.fields.latitude;
		commisioning_data.sattelite_position[2] = sat_pos.fields.altitude;
		//eslADCS_getSatelliteVelocityVec(&sat_vel); //filling the sat_vel with data
		commisioning_data.sattelite_velocity[0] = sat_vel.fields.x_velocity;
		commisioning_data.sattelite_velocity[1] = sat_vel.fields.y_velocity;
		commisioning_data.sattelite_velocity[2] = sat_vel.fields.z_velocity;
		vTaskDelay(10000 / portTICK_RATE_MS); //dealy of 10s
		WritewithEpochtime("adcs_file",0,(char *) &commisioning_data, sizeof(ADCS_comissioning_data));
	}
}

void ADC_Stage_6()
{
	int count = 0;
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_on;
	Device_ctrl.fields.motor_cubecontrol = selection_auto;
	Device_ctrl.fields.pwr_cubesense = selection_auto;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_powerdev_t Device_ctrl1;
	adcs_angrate_t Ang_rates;
	adcs_attangles_t att_angles;
	adcs_angrate_t Sen_rates;
	ADCS_comissioning_data commisioning_data;
	adcs_wheelspeed_t wheel_speed;
	adcs_wheelspeed_t wheelspeed_cmd;
	adcs_wheelspeed_t wheelspeed_cmd1;
	adcs_estmode_t mode = est_magnetometer_ratewithpitch;
	adcs_magfieldvec_t mag_field;
	adcs_ctrlmodeset_t modesetting;
	//adcs_ctrlmodeset_t modesetting1;
	wheelspeed_cmd1.fields.speedY = 0;
	modesetting.fields.mode = ctrl_mode_none;
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	eslADCS_setStateADCS(state_enabled);
	eslADCS_setEstimationMode(mode);
	eslADCS_setAttitudeCtrlMode(modesetting);
	vTaskDelay(5000 / portTICK_RATE_MS);
	eslADCS_setPwrCtrlDevice(Device_ctrl1);
	vTaskDelay(5000 / portTICK_RATE_MS);
	eslADCS_setWheelSpeed(wheelspeed_cmd); //check if there is a set function for this
	vTaskDelay(2000 / portTICK_RATE_MS);
	eslADCS_setWheelSpeed(wheelspeed_cmd1);
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	modesetting.fields.mode = ctrl_mode_detumbling;
	while (adcs_stage == 6)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getEstimatedAttAngles(&att_angles);
		commisioning_data.estimated_attitude_angles[0] = att_angles.fields.pitch;
		commisioning_data.estimated_attitude_angles[1] = att_angles.fields.roll;
		commisioning_data.estimated_attitude_angles[2] = att_angles.fields.yaw;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getWheelSpeed(&wheel_speed);
		commisioning_data.wheel_speed_estimation[0] = wheel_speed.fields.speedX;
		commisioning_data.wheel_speed_estimation[1] = wheel_speed.fields.speedY;
		commisioning_data.wheel_speed_estimation[2] = wheel_speed.fields.speedZ;
		eslADCS_getMagneticFieldVec(&mag_field);
		commisioning_data.magnetic_field_vactor[0] = mag_field.fields.x_magfield;
		commisioning_data.magnetic_field_vactor[1] = mag_field.fields.y_magfield;
		commisioning_data.magnetic_field_vactor[2] = mag_field.fields.z_magfield;
		vTaskDelay(1000 / portTICK_RATE_MS);
		count++;
		if (count == 10)
		{
			eslADCS_getMagneticFieldVec(&mag_field);
			commisioning_data.magnetic_field_vactor[0] = mag_field.fields.x_magfield;
			commisioning_data.magnetic_field_vactor[1] = mag_field.fields.y_magfield;
			commisioning_data.magnetic_field_vactor[2] = mag_field.fields.z_magfield;
		}

		WritewithEpochtime("adcs_file",0,(char *) &commisioning_data, sizeof(ADCS_comissioning_data));
	}
}

void ADC_Stage_7()
{
	adcs_powerdev_t Device_ctrl;
	Device_ctrl.fields.pwr_motor = selection_auto;
	Device_ctrl.fields.motor_cubecontrol = selection_auto;
	Device_ctrl.fields.pwr_cubesense = selection_on;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	adcs_raw_nadir_t raw_nadir;
	adcs_raw_sun_t raw_sun;
	adcs_raw_css_t raw_css;
	adcs_angrate_t Ang_rates;
	adcs_attangles_t att_angles;
	adcs_angrate_t Sen_rates;
	ADCS_comissioning_data commisioning_data;
	adcs_unixtime_t unix_time;
	eslADCS_setStateADCS(state_enabled);
	eslADCS_getCurrentTime(&unix_time);
	//eslADCS_setOrbitParam(); //need to complete
	//need to complete set estimation parameters
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	while (adcs_stage == 7)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates); //filling the Ang_rates with data
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		//eslADCS_getEstimatedAttAngles(&att_angles);
		commisioning_data.estimated_attitude_angles[0] = att_angles.fields.pitch;
		commisioning_data.estimated_attitude_angles[1] = att_angles.fields.roll;
		commisioning_data.estimated_attitude_angles[2] = att_angles.fields.yaw;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getRawCssMeasurements(&raw_css);
		commisioning_data.RAW_CSS[0] = raw_css.fields.css_1;
		commisioning_data.RAW_CSS[1] = raw_css.fields.css_2;
		commisioning_data.RAW_CSS[2] = raw_css.fields.css_3;
		commisioning_data.RAW_CSS[3] = raw_css.fields.css_4;
		commisioning_data.RAW_CSS[4] = raw_css.fields.css_5;
		commisioning_data.RAW_CSS[5] = raw_css.fields.css_6;
		//eslADCS_getRawNadirSensor(&raw_nadir);
		commisioning_data.RAW_nadir_sensors[0] = raw_nadir.fields.nadir_centroid_x;
		commisioning_data.RAW_nadir_sensors[1] = raw_nadir.fields.nadir_centroid_y;
		commisioning_data.RAW_nadir_sensors[2] = raw_nadir.fields.nadir_busystatus;
		commisioning_data.RAW_nadir_sensors[3] = raw_nadir.fields.nadir_result;
		eslADCS_getRawSunSensor(&raw_sun);
		commisioning_data.RAW_sun_sensors[0] = raw_sun.fields.sun_centroid_x;
		commisioning_data.RAW_sun_sensors[1] = raw_sun.fields.sun_centroid_y;
		commisioning_data.RAW_sun_sensors[2] = raw_sun.fields.sun_busystatus;
		commisioning_data.RAW_sun_sensors[3] = raw_sun.fields.sun_result;
		vTaskDelay(10000 / portTICK_RATE_MS); //Delay of 10s
		WritewithEpochtime("adcs_file",0,(char *) &commisioning_data, sizeof(ADCS_comissioning_data));
	}
}

void ADC_Stage_8()
{
	adcs_powerdev_t Device_ctrl;
	adcs_angrate_t Ang_rates;
	adcs_attangles_t att_angles;
	//adcsconf_ratesensor_t conf_ratesensor;
	adcs_raw_css_t raw_css;
	adcs_raw_nadir_t raw_nadir;
	ADCS_comissioning_data commisioning_data;
	adcs_raw_sun_t raw_sun;
	adcs_angrate_t Sen_rates;
	eslADCS_setStateADCS(state_enabled);
	Device_ctrl.fields.pwr_motor = selection_auto;
	Device_ctrl.fields.motor_cubecontrol = selection_auto;
	Device_ctrl.fields.pwr_cubesense = selection_on;
	Device_ctrl.fields.pwr_gpsantlna = selection_auto;
	Device_ctrl.fields.signal_cubecontrol = selection_auto;
	eslADCS_setPwrCtrlDevice(Device_ctrl);
	while (adcs_stage == 8)
	{
		commisioning_data.sid = ADC_SID;
		eslADCS_getEstimatedAngRates(&Ang_rates);
		commisioning_data.estimated_anglar_rates[0] = Ang_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.estimated_anglar_rates[1] = Ang_rates.fields.y_angrate;
		commisioning_data.estimated_anglar_rates[2] = Ang_rates.fields.z_angrate;
		eslADCS_getEstimatedAttAngles(&att_angles);
		commisioning_data.estimated_attitude_angles[0] = att_angles.fields.pitch;
		commisioning_data.estimated_attitude_angles[1] = att_angles.fields.roll;
		commisioning_data.estimated_attitude_angles[2] = att_angles.fields.yaw;
		eslADCS_getSensorRates(&Sen_rates); //filling the Sen_rates with data
		commisioning_data.sensor_rates[0] = Sen_rates.fields.x_angrate; //getting the data to the commisioning_data struct
		commisioning_data.sensor_rates[1] = Sen_rates.fields.y_angrate;
		commisioning_data.sensor_rates[2] = Sen_rates.fields.z_angrate;
		eslADCS_getRawCssMeasurements(&raw_css);
		commisioning_data.RAW_CSS[0] = raw_css.fields.css_1;
		commisioning_data.RAW_CSS[1] = raw_css.fields.css_2;
		commisioning_data.RAW_CSS[2] = raw_css.fields.css_3;
		commisioning_data.RAW_CSS[3] = raw_css.fields.css_4;
		commisioning_data.RAW_CSS[4] = raw_css.fields.css_5;
		commisioning_data.RAW_CSS[5] = raw_css.fields.css_6;
	    eslADCS_getRawNadirSensor(&raw_nadir);
	    commisioning_data.RAW_nadir_sensors[0] = raw_nadir.fields.nadir_centroid_x;
	    commisioning_data.RAW_nadir_sensors[1] = raw_nadir.fields.nadir_centroid_y;
	    commisioning_data.RAW_nadir_sensors[2] = raw_nadir.fields.nadir_busystatus;
	    commisioning_data.RAW_nadir_sensors[3] = raw_nadir.fields.nadir_result;
		eslADCS_getRawSunSensor(&raw_sun);
		commisioning_data.RAW_sun_sensors[0] = raw_sun.fields.sun_centroid_x;
		commisioning_data.RAW_sun_sensors[1] = raw_sun.fields.sun_centroid_y;
		commisioning_data.RAW_sun_sensors[2] = raw_sun.fields.sun_busystatus;
		commisioning_data.RAW_sun_sensors[3] = raw_sun.fields.sun_result;
		vTaskDelay(10000 / portTICK_RATE_MS);
		WritewithEpochtime("adcs_file",0,(char *) &commisioning_data, sizeof(ADCS_comissioning_data));;
	}
}



void task_adcs_commissioning()
{
	f_enterFS();
	unsigned char reset = 0x01;
	printf("delay for 18 seconds\n");
	I2C_write(0x12,&reset,1);
	vTaskDelay(18000 / portTICK_RATE_MS);
	adcs_stage = 1;
	while(1)
	{
		printf("enter stage %d\n",adcs_stage);
		adc_stages(adcs_stage);
	}
}


