
#ifndef ADCS_OPS
#define ADCS_OPS

#include "main.h"
#include "eslADCS_types.h"

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

#endif
