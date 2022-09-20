/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    T4.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"

#include "LCD_nokia.h"
#include "LCD_nokia_images.h"
#include "stdint.h"
#include "SPI.h"

/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

SemaphoreHandle_t xSemaphore;

uint32_t counter = 0;



extern const uint8_t ITESO[504];

void Init_LCD_task(void);

void print_LCD_TASK(void);

int main(void) {


	xSemaphore = xSemaphoreCreateBinary();

	xTaskCreate( Init_LCD_task, "Task INIT LCD", 500, NULL, 3, NULL );

	xTaskCreate( print_LCD_TASK, "Task WRITE LCD", 500, NULL, 2, NULL );



	vTaskStartScheduler();

	 int i = 0;
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

void Init_LCD_task(void)
{
	SPI_NOKIA_config();
	LCD_nokia_init(); /*! Configuration function for the LCD */

	while(1)
	{
		LCD_nokia_clear();
		counter++;
		LCD_nokia_goto_xy(0,0);
		uint8_t name[] = {"ie718817 "};
		LCD_nokia_send_string(&name[0]); /*! It print a string stored in an array*/
		xSemaphoreGive(xSemaphore);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void print_LCD_TASK(void)
{
	while(1)
	{
		if(pdTRUE == (xSemaphoreTake(xSemaphore, (TickType_t)1 )))
		{
			LCD_nokia_goto_xy(0,0);
			uint8_t name[] = {"ie718817 "};
			LCD_nokia_send_string(&name[0]); /*! It print a string stored in an array*/

			LCD_nokia_goto_xy(0,2);
			uint8_t name2[] = {"ie728442 "};
			LCD_nokia_send_string(&name2[0]); /*! It print a string stored in an array*/

			LCD_nokia_goto_xy(0,4);
			uint8_t time[] = {"00:00:01 "};
			time[8] = time[8]+1;
			LCD_nokia_send_string(&time[0]); /*! It print a string stored in an array*/

		}

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
