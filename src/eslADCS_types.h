/*
 * eslADCS_types.h
 *
 *  Created on: 2 okt. 2014
 *      Author: malv
 */

#ifndef ESLADCS_TYPES_H_
#define ESLADCS_TYPES_H_

//Telecommand parameter sizes
#define ESLADCS_GENPARAM_SIZE 	 		 	6
#define ESLADCS_CTRLPARAM_SIZE 	 		 	5
#define ESLADCS_CAPSAVEIMAGE_SIZE 	 		10
#define ESLADCS_FILENAME_SIZE 	 			13
#define ESLADCS_PROGRAMDATA_SIZE			259
#define ESLADCS_FINALPRGDATA_SIZE			71

//Telemetry reception sizes
#define ESLADCS_GENINFO_SIZE 	 	 		8
#define ESLADCS_GENDATA_SIZE 	 		 	6
#define ESLADCS_ACKSTATUS_SIZE 	 		 	4
#define ESLADCS_GENCMDS_SIZE 	 		 	12
#define ESLADCS_LOGCMDS_SIZE 	 		 	13
#define ESLADCS_PWRTEMP_SIZE	 	 		18
#define ESLADCS_EDACDATA_SIZE 	 	 		10
#define ESLADCS_FILEINFO_SIZE 	 	 		22
#define ESLADCS_FILEDATABLK_SIZE			256
#define ESLADCS_CURRSTATE_SIZE				48
#define ESLADCS_MEASUREMENTS_SIZE			36
#define ESLADCS_RAWSENSORMEASUREMENT_SIZE	60
#define ESLADCS_ESTIMATION_METADATA_SIZE	42
#define ESLADCS_ORBITPARAM_SIZE				64
#define ESLADCS_GENCONFIG_SIZE				13
#define ESLADCS_CSSCONFIG_SIZE				14
#define ESLADCS_SUNCONFIG_SIZE				17
#define ESLADCS_NADIRCONFIG_SIZE			57
#define ESLADCS_MAGMETERCONFIG_SIZE			30
#define ESLADCS_RATECONFIG_SIZE				6
#define ESLADCS_DETUMCTRLCONFIG_SIZE		10
#define ESLADCS_YMOMENTUMCONFIG_SIZE		30
#define ESLADCS_INERTIAMOMENTCONFIG_SIZE	24
#define ESLADCS_ESTIMATIONCONFIG_SIZE		26
#define ESLADCS_COMPLETECONFIG_SIZE			240
#define ESLADCS_PRGLIST_TELEMETRY_SIZE		568
#define ESLADCS_PRG_DESCRIPTION_SIZE		64


/* Interface Enumerations*/

/// Reset enumeration
typedef enum __attribute__ ((__packed__)) _adcs_resetdevice_t
{
	reset_peripherals = 1, ///< Reset MCU communications hardware and A\D unit
	reset_mcu = 2, ///< Reset MCU
	reset_adcs = 3, ///< Reset ADCS processing library
	reset_adcs_nodes = 4, ///< Send telecommand to ADCS nodes to perform a reset
} adcs_resetdevice_t;

/// Adcs state enumeration
typedef enum __attribute__ ((__packed__)) adcs_state_t
{
	state_off = 0, ///< ADCS loop is inactive
	state_enabled = 1, ///< ADCS 1Hz loop is active
	state_triggered = 2, ///< ADCS will execute control loop only when triggered
} adcs_state_t;

/// Power selection enumeration
typedef enum __attribute__ ((__packed__)) _adcs_powerselection_t
{
	selection_off = 0, ///<
	selection_on = 1, ///<
	selection_auto = 2, ///<
	selection_simulate = 3 ///<
} adcs_powerselection_t;

/// Estimation mode enumeration
typedef enum __attribute__ ((__packed__)) _adcs_estmode_t
{
	est_mode_none = 0, ///<
	est_mems_rate = 1, ///<
	est_magnetometer_rate = 2, ///<
	est_magnetometer_ratewithpitch = 3, ///<
	est_full_state_ekf = 4, ///<
	est_triad_magsun = 5, ///<
} adcs_estmode_t;

/// Control mode enumeration
typedef enum __attribute__ ((__packed__)) _adcs_controlmode_t
{
	ctrl_mode_none = 0, ///< No control
	ctrl_mode_high_initrate_det = 1, ///< High Initial Rate Detumbling
	ctrl_mode_detumbling = 2, ///< Detumbling control
	ctrl_mode_ymomentum_initial = 3, ///< Y-Momentum stabilized - Initial Pitch Acquisition
	ctrl_mode_ymomentum_steadystate = 4, ///< Y-Momentum stabilized - Steady State
} adcs_controlmode_t;

/// Startup mode enumeration values
typedef enum __attribute__ ((__packed__)) _adcs_startupmode_t
{
	startupmode_off = 0, ///< ADCS is inactive at start-up
	startupmode_autodetumble = 1, ///< ACP will automatically start the Detumbling controller
} adcs_startupmode_t;

/// Camera selection enumeration values
typedef enum __attribute__ ((__packed__)) _adcs_camera_select_t
{
	cameraselect_nadir = 0, ///< Camera selection set to nadir
	cameraselect_sun = 1, ///< Camera selection set to sun
} adcs_camera_select_t;

//Flash Firmware
/// Boot program list enumeration values
typedef enum __attribute__ ((__packed__)) _adcs_bootprglist_t
{
	bootprglist_internalflash = 1, ///< Internal flash program
	bootprglist_eeprom = 2, ///< EEPROM
	bootprglist_externalflashprg1 = 3, ///< External flash program 1
	bootprglist_externalflashprg2 = 4, ///< External flash program 2
	bootprglist_externalflashprg3 = 5, ///< External flash program 3
	bootprglist_externalflashprg4 = 6, ///< External flash program 4
	bootprglist_externalflashprg5 = 7, ///< External flash program 5
	bootprglist_externalflashprg6 = 8, ///< External flash program 6
	bootprglist_externalflashprg7 = 9, ///< External flash program 7
} adcs_bootprglist_t;

/// Program list enumeration values
typedef enum __attribute__ ((__packed__)) _adcs_prglist_t
{
	prglist_externalflashprg1 = 1, ///< External flash program 1
	prglist_externalflashprg2 = 2, ///< External flash program 2
	prglist_externalflashprg3 = 3, ///< External flash program 3
	prglist_externalflashprg4 = 4, ///< External flash program 4
	prglist_externalflashprg5 = 5, ///< External flash program 5
	prglist_externalflashprg6 = 6, ///< External flash program 6
	prglist_externalflashprg7 = 7, ///< External flash program 7
} adcs_prglist_t;

typedef enum __attribute__ ((__packed__)) _adcs_cambusystat_t
{
	camera_idle = 0,
	camera_capturewait = 1,
	camera_capturing = 2,
	camera_detecting = 3
} adcs_cambusystat_t;

typedef enum __attribute__ ((__packed__)) _adcs_nadirdetresult_t
{
	nadir_result_noerror = 0,
	nadir_result_cameratimeout = 1,
	nadir_result_sramovercurrent = 4,
	nadir_result_edges_toomany = 10,
	nadir_result_edges_notenough = 11,
	nadir_result_inversionerror = 12,
	nadir_result_badhorizonfit = 13,
	nadir_result_nodetection = 255
} adcs_nadirdetresult_t;

typedef enum __attribute__ ((__packed__)) _adcs_sundetresult_t
{
	sun_result_noerror = 0,
	sun_result_cameratimeout = 1,
	sun_result_sramovercurrent = 4,
	sun_result_sunnotfound = 20,
	sun_result_nodetection = 255
} adcs_sundetresult_t;

typedef enum __attribute__ ((__packed__)) _adcs_gps_solucionstat_t
{
	gps_solution_computed = 0,
	gps_solution_insufficientobs = 1,
	gps_solution_noconvergence = 2,
	gps_solution_singularity = 3,
	gps_solution_covtrace_exceed = 4,
	gps_solution_notyetconverged = 5,
	gps_solution_heightorvel_exceed = 6,
	gps_solution_variance_exceed = 7,
	gps_solution_largeresidual = 8,
	gps_solution_calccomparison = 9,
	gps_solution_fixedposinvalid = 10,
	gps_solution_postypeunauthorized = 11
} adcs_gps_solucionstat_t;

typedef enum __attribute__ ((__packed__)) _adcs_lastreset_t
{
	adcsreset_poweronreset = 0,
	adcsreset_brownout_regulated = 1,
	adcsreset_brownout_unregulated = 2,
	adcsreset_externalwdt = 3,
	adcsreset_externalreset = 4,
	adcsreset_wdtreset = 5,
	adcsreset_lockupsystemreset = 6,
	adcsreset_lockupreset = 7,
	adcsreset_systemrequest = 8,
	adcsreset_unknowncause = 9
} adcs_lastreset_t;

typedef enum __attribute__ ((__packed__)) _adcs_bootcause_t
{
	adcsboot_unexpected = 0,
	adcsboot_cmd_mcureset = 1,
	adcsboot_sram_latch_up = 2,
} adcs_bootcause_t;

typedef enum __attribute__ ((__packed__)) _adcs_tcack_status_t
{
	ackstatus_noerror = 0,
	ackstatus_invalid_tcid = 1,
	ackstatus_invalid_tcparamlen = 2,
	ackstatus_invalid_tcparam = 3
} adcs_tcack_status_t;

typedef enum __attribute__ ((__packed__)) _adcs_exec_waypoint_t
{
	execwayp_init = 0,
	execwayp_idle = 1,
	execwayp_senact_comm = 2,
	execwayp_adcs_update = 3,
	execwayp_peripheral_pwrcmd = 4,
	execwayp_cputemp_samp = 5,
	execwayp_image_dwl = 6,
	execwayp_image_compr = 7,
	execwayp_sav_image_sdcard = 8,
	execwayp_logging = 9,
	execwayp_log_filecompr = 10,
	execwayp_sav_log_sdcard = 11,
	execwayp_writing_flash = 12
} adcs_exec_waypoint_t;

typedef enum __attribute__ ((__packed__)) _adcs_axis_selection_t
{
	adcs_axisselect_positive_x = 0,
	adcs_axisselect_negative_x = 1,
	adcs_axisselect_positive_y = 2,
	adcs_axisselect_negative_y = 3,
	adcs_axisselect_positive_z = 4,
	adcs_axisselect_negative_z = 5,
	adcs_axisselect_notused = 6
} adcs_axis_selection_t;

typedef enum __attribute__ ((__packed__)) _adcs_magcrtl_select_t
{
	magctrl_signalmcu_measure_ctrl = 0,
	magctrl_motormcu_measure_signalmcu_ctrl = 1,
	magctrl_motormcu_measure_ctrl = 2
} adcs_magcrtl_select_t;

typedef enum __attribute__ ((__packed__)) _adcs_rot_polarity_t
{
	rotpol_posspeedcmd_increases_ymomentum = 0,
	rotpol_posspeedcmd_decreases_ymomentum = 1
} adcs_rot_polarity_t;

typedef enum __attribute__ ((__packed__)) _adcs_image_savestatus_t
{
	imgstat_noerror = 0,
	imgstat_timeout_wait_sensor = 1,
	imgstat_timeout_wait_nxtframe = 2,
	imgstat_checksum_mismatch = 3,
	imgstat_error_sd_card = 4
} adcs_image_savestatus_t;

typedef enum __attribute__ ((__packed__)) _adcs_bootstatus_t
{
	bootstatus_newselect = 0, ///<
	bootstatus_bootsucc = 1, ///<
	bootstatus_failed_attemp_1 = 2, ///<
	bootstatus_failed_attemp_2 = 3, ///<
	bootstatus_failed_attemp_3 = 4 ///<
} adcs_bootstatus_t;

typedef union __attribute__ ((__packed__)) _adcs_ctrlmodeset_t
{
	unsigned char raw[4];
	struct __attribute__ ((__packed__))
	{
		adcs_controlmode_t mode;
		//Boolean8bit override;
		unsigned short timeout;
	} fields;
} adcs_ctrlmodeset_t;

/*Telecommand data structures*/

typedef union __attribute__ ((__packed__)) _adcs_unixtime_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned int unix_time_sec; ///< time in s since 01/01/1970, 00:00 (measurment unit is [s])
		unsigned short unix_time_millsec; ///< current millisecond count (measurment unit is [ms])
	} fields;
} adcs_unixtime_t;

typedef union __attribute__ ((__packed__)) _adcs_logdata_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_LOGCMDS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned int log_selection; ///< log selection. 32 flags indicating which telemetry frames should be logged.
		unsigned short log_period; ///< log period. Set to 0 to disable logging.
		unsigned char log_compfile; ///< Use ZIP compression on Log files.
		unsigned char log_identifier[6]; ///< Log File identifier (ASCII)
	} fields;
} adcs_logdata_t;

typedef union __attribute__ ((__packed__)) _adcs_powerdev_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_CTRLPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		adcs_powerselection_t signal_cubecontrol;
		adcs_powerselection_t motor_cubecontrol;
		adcs_powerselection_t pwr_cubesense;
		adcs_powerselection_t pwr_motor;
		adcs_powerselection_t pwr_gpsantlna;
	} fields;
} adcs_powerdev_t;

typedef union __attribute__ ((__packed__)) _adcs_attangles_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short roll; ///< Roll angle (measurment unit is [deg])
		short pitch; ///< Pitch angle (measurment unit is [deg])
		short yaw; ///< Yaw angle (measurment unit is [deg])
	} fields;
} adcs_attangles_t;

typedef union __attribute__ ((__packed__)) _adcs_angrate_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_angrate; ///< x angular rate (measurment unit is [deg/s])
		short y_angrate; ///< y angular rate (measurment unit is [deg/s])
		short z_angrate; ///< z angular rate (measurment unit is [deg/s])
	} fields;
} adcs_angrate_t;

typedef union __attribute__ ((__packed__)) _adcs_wheelspeed_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short speedX; ///< X Wheel Speed (measurment unit is [rpm])
		short speedY; ///< Y Wheel Speed (measurment unit is [rpm])
		short speedZ; ///< Z Wheel Speed (measurment unit is [rpm])
	} fields;
} adcs_wheelspeed_t;

typedef union __attribute__ ((__packed__)) _adcs_magnetorq_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short magX; ///< x-torquer (duty cycle)
		short magY; ///< y-torquer (duty cycle)
		short magZ; ///< z-torquer (duty cycle)
	} fields;
} adcs_magnetorq_t;

/*Telemetry data structures*/

/** The eslADCS General Status*/
typedef union __attribute__ ((__packed__)) _adcs_geninfo_t
{
	/** Raw value array with general status data*/
	unsigned char raw[ESLADCS_GENINFO_SIZE];
	/** General Status values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char node_type;
		unsigned char version_interface;
		unsigned char version_major;
		unsigned char version_minor;
		unsigned short uptime_secs;
		unsigned short uptime_millisecs;
	} fields;
} adcs_geninfo_t;

/** The eslADCS Extended Identification*/
typedef union __attribute__ ((__packed__)) _adcs_ext_id_t
{
	/** Raw value array with extended id data*/
	unsigned char raw[ESLADCS_GENDATA_SIZE];
	/** Extended identification values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char reset_boot;
		unsigned short boot_count;
		adcs_bootprglist_t boot_prgidx;
		unsigned char firm_version_major;
		unsigned char firm_version_min;
	} fields;
} adcs_ext_id_t;

/** The eslADCS Communication Status*/
typedef union __attribute__ ((__packed__)) _adcs_commstat_t
{
	/** Raw value array with communication status data*/
	unsigned char raw[ESLADCS_GENDATA_SIZE];
	/** Communication Status values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short tc_counter;
		unsigned short tlm_reqcounter;
		unsigned char tcbuffer_overrun : 1,
		i2cTLM_readerr : 1,
		uart_protoc_err : 1,
		uart_incomp_message : 1,
		flags_reserved : 4;
		unsigned char reserved;
	} fields;
} adcs_commstat_t;

/** The eslADCS telecommand acknowledgment*/
typedef union __attribute__ ((__packed__)) _adcs_ackstat_t
{
	/** Raw value array acknowledgment status data*/
	unsigned char raw[ESLADCS_ACKSTATUS_SIZE];
	/** Acknowledgment Status values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char last_tc_id;
		unsigned char last_tc_procflag;
		adcs_tcack_status_t error_proctc;
		unsigned char index_tcerror;
	} fields;
} adcs_ackstat_t;

/** The eslADCS ACP execution state*/
typedef union __attribute__ ((__packed__)) _adcs_acploop_info_t
{
	/** Raw value array with ACP execution state data*/
	unsigned char raw[sizeof(unsigned short) + 1];
	/** ACP execution state values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short time_start;
		adcs_exec_waypoint_t curr_exec;
	} fields;
} adcs_acploop_info_t;


/** The eslADCS ACP execution times*/
typedef union __attribute__ ((__packed__)) _adcs_acptimes_info_t
{
	/** Raw value array ACP execution time data*/
	unsigned char raw[ESLADCS_GENINFO_SIZE];
	/** ACP execution time values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short adc_update;
		unsigned short senact_com;
		unsigned short exc_sgp4_prop;
		unsigned short exc_igrf_model;
	} fields;
} adcs_acptimes_info_t;

/** The eslADCS EDAC and latchup Counters*/
typedef union __attribute__ ((__packed__)) _adcs_edac_latcount_info_t
{
	/** Raw value array EDAC and Latchup Counters data*/
	unsigned char raw[ESLADCS_EDACDATA_SIZE];
	/** EDAC and Latchup Counters values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short single_sramupsets;
		unsigned short double_sramupsets;
		unsigned short multiple_sramupsets;
		unsigned short sram1_latchups;
		unsigned short sram2_latchups;
	} fields;
} adcs_edac_latcount_info_t;

/** The eslADCS file information for current file index*/
typedef union __attribute__ ((__packed__)) _adcs_file_info_t
{
	/** Raw value array File information data*/
	unsigned char raw[ESLADCS_FILEINFO_SIZE];
	/** File information values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char proc_flag : 1,
		endreach_flag : 7;
		unsigned char file_name[ESLADCS_FILENAME_SIZE];
		unsigned int file_size;
		unsigned short file_date;
		unsigned short file_time;
	} fields;
} adcs_file_info_t;

/** The eslADCS file block CRC*/
typedef union __attribute__ ((__packed__)) _adcs_file_blkcrc_t
{
	/** Raw value array file block CRC data*/
	unsigned char raw[ESLADCS_GENDATA_SIZE];
	/** File block CRC values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short blknum;
		unsigned char blklen;
		unsigned char proc_flag : 1,
		endreach_flag : 1,
		file_notfound: 1,
		read_error: 5;
		unsigned short crc16;
	} fields;
} adcs_file_blkcrc_t;

typedef union __attribute__ ((__packed__)) _adcs_pwrtemptlm_t
{
	/** Raw value array with estimation data*/
	unsigned char raw[ESLADCS_PWRTEMP_SIZE];
	/** Estimation values*/
	struct __attribute__ ((__packed__))
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
	} fields;
} adcs_pwrtemptlm_t;

typedef struct __attribute__ ((__packed__)) _adcs_currstatmode_t
{
	unsigned char adcs_state : 2,
	mode_estimation : 3,
	mode_control : 3;
} adcs_currstatmode_t;

typedef union __attribute__ ((__packed__)) _adcs_flags_t
{
	/** Raw value array with flags data*/
	unsigned char raw[ESLADCS_GENDATA_SIZE - 1];

	struct __attribute__ ((__packed__))
	{
		unsigned int cc_signal_enabled : 1,
		cc_motor_enabled : 1,
		cubesense_enabled : 1,
		gps_rx_enabled : 1,
		cc_lnapower_enabled : 1,
		motordriver_enabled : 1,
		magmeter_dep_enabled : 1,
		config_load_error : 1,
		orbit_param_error : 1,
		cs_comm_error : 1,
		cc_signal_comm_error : 1,
		cc_motor_comm_error : 1,
		cs_node_id_error : 1,
		cc_signal_id_error : 1,
		cc_motor_id_error : 1,
		magmeter_deploypower_error : 1,
		ctrl_looptime_exc : 1,
		magmeter_range_error : 1,
		sunsensor_overcurr : 1,
		sunsensor_busy_error : 1,
		sunsensor_detec_error : 1,
		wheelsensor_control_error : 1,
		nadirsensor_overcurr : 1,
		nadirsensor_busy_error : 1,
		nadirsensor_detec_error : 1,
		estimation_error : 1,
		ratesensor_range_error : 1,
		wheelsensor_range_error : 1,
		coarse_sunsensor_error : 1,
		orbitparam_invalid : 1,
		config_invalid : 1,
		ctrlmode_change_notallowed : 1
		;
		unsigned char estimator_change_notallowed : 1,
		modmeas_magdiff_size : 1,
		sram_latchup_error : 1,
		sd_card_error : 1,
		node_recov_error : 1,
		steady_state_ythomson : 1,
		triad_ekf_same : 1,
		sun_abovelocal_horizon : 1
		;
	}fields;
} adcs_flags_t;

typedef union __attribute__ ((__packed__)) _adcs_ctrlstate_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char adcs_state : 2,
		att_estimation_mode : 3,
		control_mode : 3
		;
		adcs_flags_t ctrl_adcs_flags;
	} fields;
} adcs_ctrlstate_t;

typedef union __attribute__ ((__packed__)) _adcs_currstate_t
{
	unsigned char raw[ESLADCS_CURRSTATE_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned long time_epoch;
		unsigned short time_millisecs;
		adcs_currstatmode_t current_mode;
		adcs_flags_t adcs_flags;
		short commanded_roll;
		short commanded_pitch;
		short commanded_yaw;
		short angle_roll;
		short angle_pitch;
		short angle_yaw;
		short rate_x;
		short rate_y;
		short rate_z;
		short position_x;
		short position_y;
		short position_z;
		short velocity_x;
		short velocity_y;
		short velocity_z;
		short latitude;
		short longitude;
		unsigned short altitude;
	} fields;
} adcs_currstate_t;

typedef union __attribute__ ((__packed__)) _adcs_refllhcoord_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short latitude;
		short longitud;
		short altitude;
	} fields;
} adcs_refllhcoord_t;

typedef union __attribute__ ((__packed__)) _adcs_ecirefvel_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_velocity; ///< x-velocity
		short y_velocity; ///< y-velocity
		short z_velocity; ///< z-velocity
	} fields;
} adcs_ecirefvel_t;

typedef union __attribute__ ((__packed__)) _adcs_magfieldvec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_magfield; ///< x-magnetic field (nT)
		short y_magfield; ///< y-magnetic field (nT)
		short z_magfield; ///< z-magnetic field (nT)
	} fields;
} adcs_magfieldvec_t;

typedef union __attribute__ ((__packed__)) _adcs_sunvec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_sun_cmp; ///< x-sun vector (nT)
		short y_sun_cmp; ///< y-sun vector (nT)
		short z_sun_cmp; ///< z-sun vector (nT)
	} fields;
} adcs_sunvec_t;

typedef union __attribute__ ((__packed__)) _adcs_nadirvec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_nadir; ///< x-nadir vector
		short y_nadir; ///< y-nadir vector
		short z_nadir; ///< z-nadir vector
	} fields;
} adcs_nadirvec_t;

typedef union __attribute__ ((__packed__)) _adcs_actcmds_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENCMDS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		adcs_magnetorq_t magtorquer_cmds;
		adcs_wheelspeed_t wheel_speed_cmds;
	} fields;
} adcs_actcmds_t;

typedef union __attribute__ ((__packed__)) _adcs_measurements_t
{
	/** Raw value array of data*/
	unsigned char raw[6 * ESLADCS_GENCMDS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		adcs_magfieldvec_t magfield;
		adcs_sunvec_t course_sun;
		adcs_sunvec_t fine_sun;
		adcs_nadirvec_t nadir_vector;
		adcs_angrate_t angular_rate;
		adcs_wheelspeed_t wheel_speed;
	} fields;
} adcs_measurements_t;

typedef union __attribute__ ((__packed__)) _adcs_igrf_magfield_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short igrf_magfield_x; ///< magnetic field x component
		short igrf_magfield_y; ///< magnetic field y component
		short igrf_magfield_z; ///< magnetic field z component
	} fields;
} adcs_igrf_magfield_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_sunmodel_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short sunvecmodel_x; ///< sun modeled x component
		short sunvecmodel_y; ///< sun modeled y component
		short sunvecmodel_z; ///< sun modeled z component
	} fields;
} adcs_sunmodel_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_estgyro_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short estgyrobias_x; ///< estimated rate sensor bias x component
		short estgyrobias_y; ///< estimated rate sensor bias y component
		short estgyrobias_z; ///< estimated rate sensor bias z component
	} fields;
} adcs_estgyro_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_est_innov_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short innovationvec_x; ///< estimation innovation x component
		short innovationvec_y; ///< estimation innovation y component
		short innovationvec_z; ///< estimation innovation z component
	} fields;
} adcs_est_innov_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_quaterr_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short errquaternion_q1; ///< quaternion error q1 component
		short errquaternion_q2; ///< quaternion error q2 component
		short errquaternion_q3; ///< quaternion error q3 component
	} fields;
} adcs_quaterr_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_quatcov_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short quatcovariancerms_q1; ///< quaternion covariance q1 component
		short quatcovariancerms_q2; ///< quaternion covariance q2 component
		short quatcovariancerms_q3; ///< quaternion covariance q3 component
	} fields;
} adcs_quatcov_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_angrate_cov_vec_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short angratecov_x; ///< x angular rate (measurement unit is [deg/s])
		short angratecov_y; ///< y angular rate (measurement unit is [deg/s])
		short angratecov_z; ///< z angular rate (measurement unit is [deg/s])
	} fields;
} adcs_angrate_cov_vec_t;

typedef union __attribute__ ((__packed__)) _adcs_estmetadata_t
{
	unsigned char raw[ESLADCS_ESTIMATION_METADATA_SIZE];
	struct __attribute__ ((__packed__))
	{
		adcs_igrf_magfield_vec_t igrf_magfield;
		adcs_sunmodel_vec_t sunvecmodel;
		adcs_estgyro_vec_t estgyrobias;
		adcs_est_innov_vec_t innovationvec;
		adcs_quaterr_vec_t errquaternion;
		adcs_quatcov_vec_t quatcovariancerms;
		adcs_angrate_cov_vec_t angratecov;
	} fields;
} adcs_estmetadata_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_nadir_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short nadir_centroid_x;
		short nadir_centroid_y;
		adcs_cambusystat_t nadir_busystatus;
		adcs_nadirdetresult_t nadir_result;
	} fields;
} adcs_raw_nadir_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_sun_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short sun_centroid_x;
		short sun_centroid_y;
		adcs_cambusystat_t sun_busystatus;
		adcs_sundetresult_t sun_result;
	} fields;
} adcs_raw_sun_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_css_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned char css_1;
		unsigned char css_2;
		unsigned char css_3;
		unsigned char css_4;
		unsigned char css_5;
		unsigned char css_6;
	} fields;
} adcs_raw_css_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_magmeter_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short magnetic_x;
		short magnetic_y;
		short magnetic_z;
	} fields;
} adcs_raw_magmeter_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_gps_status_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		adcs_gps_solucionstat_t gps_status;
		unsigned char gps_satellites_tracked;
		unsigned char gps_satellites_used;
		unsigned char gps_counter_xyzlog;
		unsigned char gps_counter_rangelog;
		unsigned char gps_logsetupmsg;
	} fields;
} adcs_raw_gps_status_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_gps_time_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short gps_refweeks;
		unsigned long gps_timemillisecs;
	} fields;
} adcs_raw_gps_time_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_gps_x_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_x;
		short ecef_velocity_x;
	} fields;
} adcs_raw_gps_x_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_gps_y_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_y;
		short ecef_velocity_y;
	} fields;
} adcs_raw_gps_y_t;

typedef union __attribute__ ((__packed__)) _adcs_raw_gps_z_t
{
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_z;
		short ecef_velocity_z;
	} fields;
} adcs_raw_gps_z_t;

typedef union __attribute__ ((__packed__)) _adcs_rawsensorms_t
{
	unsigned char raw[ESLADCS_RAWSENSORMEASUREMENT_SIZE];
	struct __attribute__ ((__packed__))
	{
		adcs_raw_nadir_t nadir_raw;
		adcs_raw_sun_t sun_raw;
		adcs_raw_css_t css_raw;
		adcs_raw_magmeter_t magmeter_raw;
		adcs_raw_gps_status_t gpsraw_status;
		adcs_raw_gps_time_t gpsraw_time;
		adcs_raw_gps_x_t gpsraw_ecef_x;
		adcs_raw_gps_y_t gpsraw_ecef_y;
		adcs_raw_gps_z_t gpsraw_ecef_z;

		unsigned char pos_stddev_x;
		unsigned char pos_stddev_y;
		unsigned char pos_stddev_z;

		unsigned char vel_stddev_x;
		unsigned char vel_stddev_y;
		unsigned char vel_stddev_z;
	} fields;
} adcs_rawsensorms_t;

typedef union __attribute__ ((__packed__)) _adcs_orbitparam_t
{
	unsigned char raw[ESLADCS_ORBITPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		double inclination;
		double eccentricity;
		double rightascen_ascenode;
		double arg_perigee;
		double bstar_dragterm;
		double mean_motion;
		double mean_anomaly;
		double epoch;
	} fields;
} adcs_orbitparam_t;

typedef union __attribute__ ((__packed__)) _adcsconf_magtorq_t
{
	unsigned char raw[ESLADCS_GENCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		adcs_axis_selection_t magtorq1_conf;
		adcs_axis_selection_t magtorq2_conf;
		adcs_axis_selection_t magtorq3_conf;
		unsigned short magtorqx_maxdip;
		unsigned short magtorqy_maxdip;
		unsigned short magtorqz_maxdip;
		unsigned char magtorq_maxontime;
		unsigned short magtorq_ontime_res;
		adcs_magcrtl_select_t mag_ctrl_select;
	} fields;
} adcsconf_magtorq_t;

typedef union __attribute__ ((__packed__)) _adcsconf_wheel_t
{
	unsigned char raw[ESLADCS_GENCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short wheel_mountrans_alpha_ang;
		unsigned short wheel_mountrans_gamma_ang;
		adcs_rot_polarity_t wheel_polarity;
		unsigned short wheel_inertia;
		unsigned short wheel_maxtorq;
		unsigned short wheel_maxspeed;
		unsigned char wheel_ctrlgain;
		unsigned char wheel_backup_ctrlmode;
	} fields;
} adcsconf_wheel_t;

typedef union __attribute__ ((__packed__)) _adcsconf_css_t
{
	unsigned char raw[ESLADCS_CSSCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		adcs_axis_selection_t css1_conf;
		adcs_axis_selection_t css2_conf;
		adcs_axis_selection_t css3_conf;
		adcs_axis_selection_t css4_conf;
		adcs_axis_selection_t css5_conf;
		adcs_axis_selection_t css6_conf;
		unsigned char css1_relscale;
		unsigned char css2_relscale;
		unsigned char css3_relscale;
		unsigned char css4_relscale;
		unsigned char css5_relscale;
		unsigned char css6_relscale;
		unsigned char css_autofill;
		unsigned char css_threshold;
	} fields;
} adcsconf_css_t;

typedef union __attribute__ ((__packed__)) _adcsconf_sunsensor_t
{
	unsigned char raw[ESLADCS_SUNCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short sunsensor_mountrans_alpha_ang;
		unsigned short sunsensor_mountrans_beta_ang;
		unsigned short sunsensor_mountrans_gamma_ang;
		unsigned char sundetect_thres;
		unsigned char sunsensor_auto_adjmode;
		unsigned char sunsensor_exp_time;
		unsigned char sunsensor_AGC;
		unsigned char sunblue_gain;
		unsigned char sunred_gain;
		unsigned short sun_boresightx;
		unsigned short sun_boresighty;
		unsigned char sun_shift;
	} fields;
} adcsconf_sunsensor_t;

typedef union __attribute__ ((__packed__)) _adcsconf_nadirsensor_t
{
	unsigned char raw[ESLADCS_NADIRCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short nadirsensor_mountrans_alpha_ang;
		unsigned short nadirsensor_mountrans_beta_ang;
		unsigned short nadirsensor_mountrans_gamma_ang;
		unsigned char nadirdetect_thres;
		unsigned char nadirsensor_auto_adjmode;
		unsigned char nadirsensor_exp_time;
		unsigned char nadirsensor_AGC;
		unsigned char nadirblue_gain;
		unsigned char nadirred_gain;
		unsigned short nadir_boresightx;
		unsigned short nadir_boresighty;
		unsigned char nadir_shift;
		unsigned short min_x_area1;
		unsigned short max_x_area1;
		unsigned short min_y_area1;
		unsigned short max_y_area1;
		unsigned short min_x_area2;
		unsigned short max_x_area2;
		unsigned short min_y_area2;
		unsigned short max_y_area2;
		unsigned short min_x_area3;
		unsigned short max_x_area3;
		unsigned short min_y_area3;
		unsigned short max_y_area3;
		unsigned short min_x_area4;
		unsigned short max_x_area4;
		unsigned short min_y_area4;
		unsigned short max_y_area4;
		unsigned short min_x_area5;
		unsigned short max_x_area5;
		unsigned short min_y_area5;
		unsigned short max_y_area5;
	} fields;
} adcsconf_nadirsensor_t;

typedef union __attribute__ ((__packed__)) _adcsconf_magmeter_t
{
	unsigned char raw[ESLADCS_MAGMETERCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short magtorq_mountrans_alpha_ang;
		unsigned short magtorq_mountrans_beta_ang;
		unsigned short magtorq_mountrans_gamma_ang;
		unsigned short magtorq_ch1_offset;
		unsigned short magtorq_ch2_offset;
		unsigned short magtorq_ch3_offset;
		unsigned short magtorq_senmatrix_s11;
		unsigned short magtorq_senmatrix_s12;
		unsigned short magtorq_senmatrix_s13;
		unsigned short magtorq_senmatrix_s21;
		unsigned short magtorq_senmatrix_s22;
		unsigned short magtorq_senmatrix_s23;
		unsigned short magtorq_senmatrix_s31;
		unsigned short magtorq_senmatrix_s32;
		unsigned short magtorq_senmatrix_s33;
	} fields;
} adcsconf_magmeter_t;

typedef union __attribute__ ((__packed__)) _adcsconf_ratesensor_t
{
	unsigned char raw[ESLADCS_RATECONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short yrate_sensor_offset;
		unsigned short yrate_mountrans_alpha_ang;
		unsigned short yrate_mountrans_gamma_ang;
	} fields;
} adcsconf_ratesensor_t;

typedef union __attribute__ ((__packed__)) _adcsconf_detumctrl_t
{
	unsigned char raw[ESLADCS_DETUMCTRLCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned int detumb_spingain;
		unsigned int detumb_dampgain;
		unsigned short ref_spinrate;
	} fields;
} adcsconf_detumctrl_t;

typedef union __attribute__ ((__packed__)) _adcsconf_ymomentum_ctrl_t
{
	unsigned char raw[ESLADCS_YMOMENTUMCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned int ymomentum_ctrl_gain;
		unsigned int ymomentum_devnuta_dampgain;
		unsigned int ymomentum_propnuta_dampgain;
		unsigned int ymomentum_propgain;
		unsigned int ymomentum_devgain;
		unsigned int ref_wheel_moment;
		unsigned int ref_wheel_moment_initpitch;
		unsigned short ref_wheel_torq_initpitch;
	} fields;
} adcsconf_ymomentum_ctrl_t;

typedef union __attribute__ ((__packed__)) _adcs_moment_inertia_conf_t
{
	unsigned char raw[ESLADCS_INERTIAMOMENTCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned int moment_inertia_lxx;
		unsigned int moment_inertia_lyy;
		unsigned int moment_inertia_lzz;
		unsigned int moment_inertia_lxy;
		unsigned int moment_inertia_lxz;
		unsigned int moment_inertia_lyz;
	} fields;
} adcsconf_moment_inertia_t;

typedef union __attribute__ ((__packed__)) _adcs_sensor_masks_t
{
	/** Raw value mask data*/
	unsigned char raw;

	struct __attribute__ ((__packed__))
	{
		unsigned char mask_sunsensor : 1,
		mask_nadirsensor : 1,
		mask_css : 6
		//mask_ratesensor : 5
		;
	}fields;
} adcs_sensor_masks_t;

typedef union __attribute__ ((__packed__)) _adcsconf_estimation_t
{
	unsigned char raw[ESLADCS_ESTIMATIONCONFIG_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned int magtorq_ratefilter_sysnoise;
		unsigned int ekf_sysnoise;
		unsigned int css_measurenoise;
		unsigned int sunsensor_measurenoise;
		unsigned int nadirsensor_measurenoise;
		unsigned int magmeter_measurenoise;
		adcs_sensor_masks_t sensor_mask;
		unsigned char sun_nadir_samperiod;
	} fields;
} adcsconf_estimation_t;

typedef union __attribute__ ((__packed__)) _adcsconf_full_t
{
	/** Raw value mask data*/
	unsigned char raw[ESLADCS_COMPLETECONFIG_SIZE];

	struct __attribute__ ((__packed__))
	{
		adcsconf_magtorq_t conf_magtorq;
		adcsconf_wheel_t conf_wheel;
		adcsconf_css_t conf_css;
		adcsconf_sunsensor_t conf_sun;
		adcsconf_nadirsensor_t conf_nadir;
		adcsconf_magmeter_t conf_magmeter;
		adcsconf_ratesensor_t conf_ratesensor;
		adcsconf_detumctrl_t conf_detumctrl;
		adcsconf_ymomentum_ctrl_t conf_ymomentum;
		adcsconf_moment_inertia_t conf_mominertia;
		adcsconf_estimation_t conf_estparam;
	}fields;
} adcsconf_full_t;

typedef union __attribute__ ((__packed__)) _adcs_capsave_image_t
{
	/** Raw value mask data*/
	unsigned char raw[ESLADCS_CAPSAVEIMAGE_SIZE];

	struct __attribute__ ((__packed__))
	{
		adcs_camera_select_t conf_magtorq;
		unsigned char capture_newimage : 1,
		compress_imageusing_jpg : 7;
		unsigned char image_identifier[ESLADCS_CAPSAVEIMAGE_SIZE - 2];
	}fields;
} adcs_capsave_image_t;

typedef union __attribute__ ((__packed__)) _adcs_uploadblock_t
{
	/** Raw value mask data*/
	unsigned char raw[ESLADCS_PROGRAMDATA_SIZE];

	struct __attribute__ ((__packed__))
	{
		adcs_prglist_t prglist;
		unsigned short block_number;
		unsigned char prgdata[ESLADCS_PROGRAMDATA_SIZE - 3];
	}fields;
} adcs_uploadblock_t;

typedef union __attribute__ ((__packed__)) _adcs_finalize_upload_t
{
	/** Raw value mask data*/
	unsigned char raw[ESLADCS_FINALPRGDATA_SIZE];

	struct __attribute__ ((__packed__))
	{
		adcs_prglist_t prglist;
		unsigned int prg_length;
		unsigned short crc16;
		unsigned char prgdesc[ESLADCS_FINALPRGDATA_SIZE - 7];
	}fields;
} adcs_finalize_upload_t;

typedef union __attribute__ ((__packed__)) _adcs_csense_current_meas_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short cs_3v3_current; ///< CubeSense 3V3 current
		char cs_nadir_sram_current; ///< CubeSense Nadir SRAM current
		char cs_sun_sram_current; ///< CubeSense Sun SRAM current
		short arm_cpu_temp; ///< ARM CPU Temperature
	} fields;
} adcs_csense_current_meas_t;

typedef union __attribute__ ((__packed__)) _adcs_ccontrol_current_meas_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short cc_3v3_current; ///< CubeControl 3V3 current
		short cc_5v_current; ///< CubeControl 5V current
		short cc_Vbat_current; ///< CubeControl Vbat current
	} fields;
} adcs_ccontrol_current_meas_t;

typedef union __attribute__ ((__packed__)) _adcs_percurr_temp_t
{
	/** Raw value array of data*/
	unsigned char raw[ESLADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short magnetorquer_current; ///< Magnetorquer current
		short momentum_wheel_current; ///< Momentum Wheel current
		char rate_sensor_temp; ///< Rate Sensor temperature
		char magnetorquer_temp; ///< Magnetorquer temperature
	} fields;
} adcs_percurr_temp_t;

typedef union __attribute__ ((__packed__)) _adcs_status_capsave_image_t
{
	/** Raw value mask data*/
	unsigned char raw[2];

	struct __attribute__ ((__packed__))
	{
		unsigned char perc_complete;
		adcs_image_savestatus_t curr_status;
	}fields;
} adcs_status_capsave_image_t;

typedef union __attribute__ ((__packed__)) _adcs_uploaded_prg_status_t
{
	/** Raw value mask data*/
	unsigned char raw[sizeof(unsigned short) + 1];

	struct __attribute__ ((__packed__))
	{
		unsigned char prg_complete;
		unsigned short crc16_checksum;
	}fields;
} adcs_uploaded_prg_status_t;

typedef union __attribute__ ((__packed__)) _adcs_prglist_telem_t
{
	/** Raw value mask data*/
	unsigned char raw[ESLADCS_PRGLIST_TELEMETRY_SIZE];

	struct __attribute__ ((__packed__))
	{
		unsigned int prg_1_length;
		unsigned short prg_1_crc16;
		unsigned char prg_1_valid;
		unsigned char prg_1_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_2_length;
		unsigned short prg_2_crc16;
		unsigned char prg_2_valid;
		unsigned char prg_2_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_3_length;
		unsigned short prg_3_crc16;
		unsigned char prg_3_valid;
		unsigned char prg_3_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_4_length;
		unsigned short prg_4_crc16;
		unsigned char prg_4_valid;
		unsigned char prg_4_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_5_length;
		unsigned short prg_5_crc16;
		unsigned char prg_5_valid;
		unsigned char prg_5_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_6_length;
		unsigned short prg_6_crc16;
		unsigned char prg_6_valid;
		unsigned char prg_6_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int prg_7_length;
		unsigned short prg_7_crc16;
		unsigned char prg_7_valid;
		unsigned char prg_7_description[ESLADCS_PRG_DESCRIPTION_SIZE];
		unsigned int intflash_prg_length;
		unsigned short intflash_prg_crc16;
		unsigned char intflash_prg_valid;
		unsigned char intflash_prg_description[ESLADCS_PRG_DESCRIPTION_SIZE];
	}fields;
} adcs_prglist_telem_t;

typedef union __attribute__ ((__packed__)) _adcs_boot_idx_telem_t
{
	/** Raw value mask data*/
	unsigned char raw[2];

	struct __attribute__ ((__packed__))
	{
		adcs_bootprglist_t prg_idx;
		adcs_bootstatus_t curr_bootstat;
	}fields;
} adcs_boot_idx_telem_t;

#endif /* ESLADCS_TYPES_H_ */
