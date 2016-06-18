/*
 * maininclude.h
 *
 *  Created on: 21 באפר 2016
 *      Author: USER1
 */

#ifndef MAININCLUDE_H_
#define MAININCLUDE_H_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <hal/Timing/WatchDogTimer.h>
#include <hal/Drivers/LED.h>
#include <hal/boolean.h>

#include <hal/utility/util.h>
#include <hal/Timing/Time.h>
#include <hal/Timing/RTC.h>

#include <at91/utility/trace.h>
#include <at91/peripherals/cp15/cp15.h>
#include <at91/utility/exithandler.h>
#include <at91/commons.h>
#include <hal/Drivers/I2C.h>
#include <stdlib.h>
#include <hcc/api_hcc_mem.h>
#include <hcc/api_fat.h>
#include <hcc/api_fat_test.h>
#include <hcc/api_mdriver_atmel_mcipdc.h>


#include "IsisAntS.h"
#include "EPS.h"
#include "GomEPS.h"
#include "trxvu.h"
#include "ADCS_operations.h"
#include "ADCS_Thread.h"
#include "eslADCS_types.h"
#include "Global.h"
#include "FileSys.h"
#include "scs.h"
#include "OurTime.h"
#include "FRAM.h"
#include <hal/Timing/Time.h>
#include "IsisTRXVU.h"
#include "Time.h"




extern unsigned char frame_count;
extern unsigned char tc_count;

#endif /* MAININCLUDE_H_ */
