/*
 * test.c
 *
 *  Created on: 5 במאי 2016
 *      Author: LAVIAN
 */

/*
 * UARTtest.c
 *
 *  Created on: 20-Feb-2013
 *      Author: Akhil Piplani
 */

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
#include "EPS.h"


#define GPIO_04	PIN_GPIO04
#define GPIO_05	PIN_GPIO05
#define GPIO_06	PIN_GPIO06
#define GPIO_07 PIN_GPIO07


void taskUARTtest(void *arguments) {
	int retValInt = 0;
	unsigned int readSize = 174, i;
	unsigned char readData[174] = {0}, writeData[3] = {0x06,0x01,0x01};
	UARTbus bus = *((UARTbus*)arguments);

	Pin Pin04 = GPIO_04;
	Pin Pin05 = GPIO_05;
	Pin Pin06 = GPIO_06;
	Pin Pin07 = GPIO_07;

	printf("Uart test\n ");
	retValInt = UART_write(bus, writeData,3); // Write 2 bytes more than we received for \n\r
	while(1)
	{
		printf("written command to mNLP, returned %d\n ",retValInt);

			if(retValInt != 0) {
				TRACE_WARNING("\n\r taskUARTtest: UART_write returned: %d for bus %d \n\r", retValInt, bus);
			}

			retValInt = UART_read(bus, readData, readSize);

			for (i=0;i<readSize;i++)
			{
				if(i==0&&readData[i]==0xBB)
				{
					PIO_Clear(&Pin04);
					vTaskDelay(10);
					PIO_Clear(&Pin05);
					vTaskDelay(10);
					PIO_Clear(&Pin06);
					vTaskDelay(10);
					PIO_Clear(&Pin07);
					vTaskDelay(10);

					//retValInt = test_da_power_OFF();
					printf("shitshitshitshit");
				}
				printf("%d: %x \n",i,readData[i]);
			}
			vTaskDelay(1000);

	}
	if (0)
	{
			vTaskDelay(50);
			for(i=0;i<1;i++)
			{
			retValInt = UART_read(bus, readData, readSize);
					if(retValInt != 0) {
						TRACE_WARNING("\n\r taskUARTtest: UART_read returned: %d for bus %d \n\r", retValInt, bus);


					}

				vTaskDelay(1);
				printf("read packet number %d, first byte: %x , second: %x, third %x",i,readData[0],readData[1],readData[2]);
			}


	}

}


Boolean test_uart_payload() {
	int retValInt = 0;
	xTaskHandle taskUART0testHandle;
	static UARTbus UARTtestBus[2] = {bus0_uart, bus2_uart};
	Pin Pin04 = GPIO_04;
	Pin Pin05 = GPIO_05;
	Pin Pin06 = GPIO_06;
	Pin Pin07 = GPIO_07;


	UARTconfig configBus0 = {.mode = AT91C_US_USMODE_NORMAL | AT91C_US_CLKS_CLOCK | AT91C_US_CHRL_8_BITS | AT91C_US_PAR_NONE | AT91C_US_OVER_16 | AT91C_US_NBSTOP_1_BIT,
								.baudrate = 9600, .timeGuard = 1, .busType = rs232_uart, .rxtimeout = 0xFFFF};

	printf("\n tring sending staff to the mnLP \n");

	// set GPIO PINS
	PIO_Set(&Pin04);
	vTaskDelay(10);
	PIO_Set(&Pin05);
	vTaskDelay(10);
	PIO_Set(&Pin06);
	vTaskDelay(10);
	PIO_Set(&Pin07);
	vTaskDelay(10);


	// Both UART peripherals must be started separately as they can use different configurations.
	//retValInt = UART_start(bus0_uart, configBus0);

	if(retValInt != 0) {
		TRACE_WARNING("\n\r UARTtest: UART_start returned %d! \n\r", retValInt);
		while(1);
	}
	//retValInt = UART_start(bus2_uart, configBus2);
	if(retValInt != 0) {
		TRACE_WARNING("\n\r UARTtest: UART_start returned %d! \n\r", retValInt);
		while(1);
	}
	printf("started");
	// Instantiate two separate versions of taskUARTtest and pass different bus-id's as a parameter.
	xTaskGenericCreate(taskUARTtest, (const signed char*)"taskUARTtest-0", 1024, (void*)&UARTtestBus[0], 2, &taskUART0testHandle, NULL, NULL);
	//xTaskGenericCreate(taskUARTtest, (const signed char*)"taskUARTtest-2", 1024, (void*)&UARTtestBus[1], 2, &taskUART2testHandle, NULL, NULL);
	while(1)
	{
		vTaskDelay(1000);
	}
	return FALSE;
}


Boolean test_da_power()
{
	unsigned int choice;
		Pin Pin04 = GPIO_04;
		Pin Pin05 = GPIO_05;
		Pin Pin06 = GPIO_06;
		Pin Pin07 = GPIO_07;


		printf("\n\r Start Test Test: \n\r");

		PIO_Configure(&Pin04, PIO_LISTSIZE(&Pin04));
		if(!PIO_Configure(&Pin04, PIO_LISTSIZE(Pin04))) {
			printf(" PinTest: Unable to configure PIOA pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin05, PIO_LISTSIZE(&Pin05));
		if(!PIO_Configure(&Pin05, PIO_LISTSIZE(Pin05))) {
			printf(" PinTest: Unable to configure PIOB pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin06, PIO_LISTSIZE(&Pin06));
		if(!PIO_Configure(&Pin06, PIO_LISTSIZE(Pin06))) {
			printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin07, PIO_LISTSIZE(&Pin07));
				if(!PIO_Configure(&Pin07, PIO_LISTSIZE(Pin06))) {
					printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
					while(1);
				}

				vTaskDelay(10);

		printf("\n\r PinTest: All pins should now be logic-0 (0V). Please check their states now. \n\r");
		printf(" PinTest: Press 1 then Enter when done. \n\r");
		//while(1);

		PIO_Set(&Pin04);
		vTaskDelay(10);
		PIO_Set(&Pin05);
		vTaskDelay(10);
		PIO_Set(&Pin06);
		vTaskDelay(10);
		PIO_Set(&Pin07);
		vTaskDelay(10);
		if (0)
		{
		vTaskDelay(5000 / portTICK_RATE_MS);
		PIO_Clear(&Pin04);
		vTaskDelay(10);
		PIO_Clear(&Pin05);
		vTaskDelay(10);
		PIO_Clear(&Pin06);
		vTaskDelay(10);
		PIO_Clear(&Pin07);
		vTaskDelay(10);

		printf("\n\r PinTest: All pins should now be logic-1 (3.3V). Please check their states now. \n\r");
		printf(" PinTest: Press 1 then Enter when done. \n\r");
		UTIL_DbguGetInteger(&choice);
		}
		while (1)
		{
			vTaskDelay(2000);
		}
		return TRUE;
}
int test_da_power_OFF()
{

		Pin Pin04 = GPIO_04;
		Pin Pin05 = GPIO_05;
		Pin Pin06 = GPIO_06;
		Pin Pin07 = GPIO_07;


		printf("\n\r Start Test Test: \n\r");

		PIO_Configure(&Pin04, PIO_LISTSIZE(&Pin04));
		if(!PIO_Configure(&Pin04, PIO_LISTSIZE(Pin04))) {
			printf(" PinTest: Unable to configure PIOA pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin05, PIO_LISTSIZE(&Pin05));
		if(!PIO_Configure(&Pin05, PIO_LISTSIZE(Pin05))) {
			printf(" PinTest: Unable to configure PIOB pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin06, PIO_LISTSIZE(&Pin06));
		if(!PIO_Configure(&Pin06, PIO_LISTSIZE(Pin06))) {
			printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
			while(1);
		}

		vTaskDelay(10);

		PIO_Configure(&Pin07, PIO_LISTSIZE(&Pin07));
				if(!PIO_Configure(&Pin07, PIO_LISTSIZE(Pin06))) {
					printf(" PinTest: Unable to configure PIOC pins as output! \n\r");
					while(1);
				}

				vTaskDelay(10);

		printf("\n\r PinTest: All pins should now be logic-0 (0V). Please check their states now. \n\r");
		printf(" PinTest: Press 1 then Enter when done. \n\r");
		//while(1);


		vTaskDelay(5000 / portTICK_RATE_MS);
		PIO_Clear(&Pin04);
		vTaskDelay(10);
		PIO_Clear(&Pin05);
		vTaskDelay(10);
		PIO_Clear(&Pin06);
		vTaskDelay(10);
		PIO_Clear(&Pin07);
		vTaskDelay(10);


		return TRUE;
}


