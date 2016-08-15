
#ifndef ADCS_OPS
#define ADCS_OPS

#include "main.h"
#include "eslADCS_types.h"
#include "ADCS_Thread.h"

typedef struct ADCS_Payload_Telametry
{
	int   epoch_time;
	short estimated_anglar_rates[3];
	short estimated_attitude_angles[3];
	short current_Position[3];
	short demo;
}ADCS_Payload_Telemetry;

typedef struct ADCS_telemetry_data
{
	char sid;
	char ratesensor_temp;
	char magnetometer_temp;
	unsigned char csense_nadirSRAMcurr;
	unsigned char csense_sunSRAMcurr;
	unsigned char stage;
	unsigned char status[6];
	unsigned short csense_3v3curr;
	unsigned short arm_cpuTemp;
	unsigned short ccontrol_3v3curr;
	unsigned short ccontrol_5Vcurr;
	unsigned short ccontrol_Vbatcurr;
	unsigned short magtorquer_curr;
	unsigned short momentum_wheelcurr;

}ADCS_telemetry_data;

typedef struct adcs_calibration
{

   char mtq[3];
   short mtq_max_mnt[3];
   short wheel_mnt_ang[2];
   short mtq_mnt_ang[3];
   short mtq_chanel[3];
   short mtq_sens_matrix[3];
   short Nadir_sensor_mnt_ang[3];
   short sun_sensor_mnt_ang[3];
   char css[6];
   short Rate_sensor_mnt_ang[2];
}adcs_calibration;


void eslADCS_getEstimatedAngRates(adcs_angrate_t* ang_rates);
void eslADCS_getSensorRates(adcs_angrate_t* sen_rates);
void eslADCS_setAttitudeCtrlMode(adcs_ctrlmodeset_t modesettings);
void eslADCS_getMagneticFieldVec(adcs_magfieldvec_t* mag_field);
void eslADCS_getMagnetorquerCmd(adcs_magnetorq_t* mag_cmd);
void eslADCS_deployMagnetometer(unsigned char act_timeout);
void eslADCS_setConfSave();
void eslADCS_getWheelSpeed(adcs_wheelspeed_t* wheel_speed);
void eslADCS_getWheelSpeed_tempCmd();
void eslADCS_getRawSunSensor(adcs_raw_sun_t* raw_sun);
void eslADCS_getRawCssMeasurements(adcs_raw_css_t* raw_css);
void eslADCS_getCurrentTime(adcs_unixtime_t* unix_time);
void eslADCS_getPwrTempTlm(adcs_pwrtemptlm_t* pwrtemp_tlm);
void ADCS_command(unsigned char id, unsigned char* data, unsigned int dat_len);
void eslADCS_telemetry_Time_Power_temp();
void eslADCS_Magnetometer_Boom_Deployment_Enabled(Boolean* Magnetometer_Status);
void eslADCS_getRawMagnetometerMeas(adcs_raw_magmeter_t* raw_mag);
void eslADCS_setStateADCS(adcs_state_t current_status);
void eslADCS_setEstimationMode(adcs_estmode_t mode);
void eslADCS_setPwrCtrlDevice(adcs_powerdev_t device_ctrl);
void eslADCS_getEstimatedAttAngles(adcs_attangles_t *att_angles);
void eslADCS_getCalibration(adcs_calibration *calibration);
void print_calibration(adcs_calibration *calibration);
void eslADCS_setOrbitParam(unsigned char* orbit_param);
void Build_PayloadPacket(unsigned char *packet);
void eslADCS_getCurrentPosition(adcs_currstate_t* current_state);
void eslADCS_getRawNadirSensor(adcs_raw_nadir_t *raw_nadir);
void eslADCS_setWheelSpeed(adcs_wheelspeed_t cmd_speed);
void idtlm();
void ADCS_get_status(unsigned char *status);
void ADCS_set_magnetometer_config();
void ADCS_update_unix_time(unsigned long t);
void ADCS_update_tle(unsigned char* tle);
void get_sat_llh_pos(adcs_refllhcoord_t *llh_in);
void adcs_reset(unsigned char type);
void adcs_set_estimation_param();
void eslADCS_getCalNadirSensor();
void eslADCS_getCalSunSensor();
void print_send_ADCS_telemetry_packet(ADCS_telemetry_data telemetry_data);
void print_payload_header(adcs_angrate_t ang_rates, adcs_attangles_t att_angles,adcs_currstate_t current_state,unsigned long t);
void test_commissioning_packet();
#endif
