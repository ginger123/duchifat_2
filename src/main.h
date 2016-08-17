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
#include <hal/Drivers/UART.h>
#include <stdlib.h>
#include <hcc/api_hcc_mem.h>
#include <hcc/api_fat.h>
#include <hcc/api_fat_test.h>
#include <hcc/api_mdriver_atmel_mcipdc.h>



//
#include <at91/boards/ISIS_OBC_G20/board.h>
#include <at91/utility/trace.h>
#include <at91/commons.h>
#include <at91/peripherals/pio/pio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/projdefs.h>

#include <hal/Drivers/UART.h>
#include <hal/interruptPriorities.h>
#include <hal/boolean.h>
#include <hal/Utility/util.h>

#include <string.h>
#include <stdio.h>
//

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
#include "mnlp.h"

// define FRAM addresses

#define THREAD_TIMESTAMP_LEN 5
extern unsigned char frame_count;
extern unsigned char tc_count;
extern unsigned long timestamp[THREAD_TIMESTAMP_LEN];//0=main 1=mnlp 2=mnlplistener 3=adcs 4=reset
extern gom_eps_channelstates_t channels_state;

#endif /* MAININCLUDE_H_ */
