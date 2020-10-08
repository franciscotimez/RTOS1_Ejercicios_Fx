/*=============================================================================
 * Copyright (c) 2020, Francisco Timez <franciscotimez@gmail.com>
 * All rights reserved.
 * License: mit (see LICENSE.txt)
 * Date: 2020/10/07
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

//#include "RTOS1_EJ_F1.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "sapi.h"
#include "keys.h"

/*=====[Definition & macros of public constants]==============================*/

/*=====[Definitions of extern global functions]==============================*/

// Prototipo de funcion de la tarea
void task_led( void* taskParmPtr );
void task_tecla( void* taskParmPtr );

/*=====[Definitions of public global variables]==============================*/
gpioMap_t leds[] = {LEDB, LED1, LED2, LED3};

/*=====[Main function, program entry point after power on or reset]==========*/

int main( void )
{
	BaseType_t res;

	// ---------- CONFIGURACIONES ------------------------------
	boardConfig();  // Inicializar y configurar la plataforma

	printf( "Ejercicio D1\n" );

	// Crear tareas en freeRTOS
	for(uint32_t i = TEC1_INDEX ; i <= TEC4_INDEX ; i++)
	{
		res = xTaskCreate (
				task_led,					// Funcion de la tarea a ejecutar
				( const char * )"task_led",	// Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2,	// Cantidad de stack de la tarea
				i,							// Parametros de tarea
				tskIDLE_PRIORITY+1,			// Prioridad de la tarea
				0							// Puntero a la tarea creada en el sistema
		);

		// Gestión de errores
		configASSERT( res == pdPASS );
	}
	/* inicializo driver de teclas */
	keys_Init();

	// Iniciar scheduler
	vTaskStartScheduler();					// Enciende tick | Crea idle y pone en ready | Evalua las tareas creadas | Prioridad mas alta pasa a running

	/* realizar un assert con "false" es equivalente al while(1) */
	configASSERT( 0 );
	return 0;
}

void task_led( void* taskParmPtr )
{
	uint32_t index = (uint32_t) taskParmPtr;
	TickType_t dif =   pdMS_TO_TICKS( 500 );
	int tecla_presionada;
	TickType_t xPeriodicity = pdMS_TO_TICKS( 1000 ); // Tarea periodica cada 1000 ms

	TickType_t xLastWakeTime = xTaskGetTickCount();

	while( 1 )
	{
		if( key_pressed( index ) )
		{
			dif = get_diff(index);
		}

		gpioWrite( leds[index], ON );
		vTaskDelay( dif );
		gpioWrite( leds[index], OFF );


		// Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
		vTaskDelayUntil( &xLastWakeTime, 2*dif );
	}
}

/* hook que se ejecuta si al necesitar un objeto dinamico, no hay memoria disponible */
void vApplicationMallocFailedHook()
{
	printf( "Malloc Failed Hook!\n" );
	configASSERT( 0 );
}
