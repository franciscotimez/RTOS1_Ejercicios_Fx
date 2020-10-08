/*
 * keys.c
 *
 *  Created on: 7 oct. 2020
 *      Author: francisco
 */


/*==================[ Inclusions ]============================================*/
#include "FreeRTOS.h"
#include "task.h"
#include "sapi.h"
#include "keys.h"

/*=====[ Definitions of private data types ]===================================*/

/*=====[Definition macros of private constants]==============================*/
//#define BUTTON_RATE     1
#define DEBOUNCE_TIME   40

/*=====[Prototypes (declarations) of private functions]======================*/
static void keys_isr_config( void );
static void keys_ButtonError( uint32_t index );
static void buttonPressed( uint32_t index );
static void buttonReleased( uint32_t index );

/*=====[Definitions of private global variables]=============================*/

/*=====[Definitions of public global variables]==============================*/

const t_key_config  keys_config[] = { [TEC1_INDEX]= {TEC1}, [TEC2_INDEX]= {TEC2}, [TEC3_INDEX]= {TEC3}, [TEC4_INDEX]= {TEC4} }  ;

#define key_count   sizeof(keys_config)/sizeof(keys_config[TEC1_INDEX])

t_key_data keys_data[key_count];

#if KEYS_USE_ISR==1
SemaphoreHandle_t isr_signal;   //almacenara el handle del semaforo creado para una cierta tecla
#endif

/*=====[prototype of private functions]=================================*/
void task_tecla( void* taskParmPtr );

/*=====[Implementations of public functions]=================================*/
TickType_t get_diff(uint32_t index)
{
	TickType_t tiempo;

	taskENTER_CRITICAL();
	tiempo = keys_data[index].time_diff;
	taskEXIT_CRITICAL();

	return tiempo;
}

void clear_diff(uint32_t index)
{
	taskENTER_CRITICAL();
	keys_data[index].time_diff = 0;
	taskEXIT_CRITICAL();
}

/* funcion no bloqueante que consulta si la tecla fue pulsada. */
int key_pressed( uint32_t index )
{
	BaseType_t signaled = xSemaphoreTake( keys_data[index].pressed_signal, 0 );
	if ( signaled == pdPASS )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void keys_Init( void )
{
	BaseType_t res;

	//extender para N teclas
	for(uint32_t i = TEC1_INDEX ; i <= TEC4_INDEX ; i++)
	{
		keys_data[i].state          = BUTTON_UP;  // Set initial state
		keys_data[i].time_down      = KEYS_INVALID_TIME;
		keys_data[i].time_up        = KEYS_INVALID_TIME;
		keys_data[i].time_diff      = KEYS_INVALID_TIME;

		keys_data[i].isr_signal    = xSemaphoreCreateBinary();
		keys_data[i].pressed_signal    = xSemaphoreCreateBinary();

		configASSERT( keys_data[i].isr_signal != NULL );
		configASSERT( keys_data[i].pressed_signal != NULL );
	}

	// Crear tareas en freeRTOS
	for(uint32_t i = TEC1_INDEX ; i <= TEC4_INDEX ; i++)
	{
		res = xTaskCreate (
				task_tecla,					// Funcion de la tarea a ejecutar
				( const char * )"task_tecla",	// Nombre de la tarea como String amigable para el usuario
				configMINIMAL_STACK_SIZE*2,	// Cantidad de stack de la tarea
				i,							// Parametros de tarea
				tskIDLE_PRIORITY+1,			// Prioridad de la tarea
				0							// Puntero a la tarea creada en el sistema
		);
	}
#if KEYS_USE_ISR==1
	keys_isr_config();
#endif

	// Gestión de errores
	configASSERT( res == pdPASS );
}

#if KEYS_USE_ISR==0


// keys_ Update State Function
void keys_Update( uint32_t index )
{
	switch( keys_data[index].state )
	{
	case STATE_BUTTON_UP:
		/* CHECK TRANSITION CONDITIONS */
		if( !gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_FALLING;
		}
		break;

	case STATE_BUTTON_FALLING:
		/* ENTRY */

		/* CHECK TRANSITION CONDITIONS */
		if( !gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_DOWN;

			/* ACCION DEL EVENTO !*/
			buttonPressed( index );
		}
		else
		{
			keys_data[index].state = STATE_BUTTON_UP;
		}

		/* LEAVE */
		break;

	case STATE_BUTTON_DOWN:
		/* CHECK TRANSITION CONDITIONS */
		if( gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_RISING;
		}
		break;

	case STATE_BUTTON_RISING:
		/* ENTRY */

		/* CHECK TRANSITION CONDITIONS */

		if( gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_UP;

			/* ACCION DEL EVENTO ! */
			buttonReleased( index );
		}
		else
		{
			keys_data[index].state = STATE_BUTTON_DOWN;
		}

		/* LEAVE */
		break;

	default:
		keys_ButtonError( index );
		break;
	}
}
#else


void keys_Update_Isr( uint32_t index )
{
	switch( keys_data[index].state )
	{
	case STATE_BUTTON_UP:

		/* espero una señal de la tecla */
		xSemaphoreTake( keys_data[index].isr_signal, portMAX_DELAY );

		/* la tecla se pulso */
		keys_data[index].state = STATE_BUTTON_FALLING;
		break;

	case STATE_BUTTON_FALLING:
		/* ENTRY */

		/* CHECK TRANSITION CONDITIONS */
		if( !gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_DOWN;

			/* ACCION DEL EVENTO !*/
			buttonPressed( index );
		}
		else
		{
			keys_data[index].state = STATE_BUTTON_UP;
		}

		/* LEAVE */
		break;

	case STATE_BUTTON_DOWN:

		/* espero una señal de la tecla */
		xSemaphoreTake( keys_data[index].isr_signal, portMAX_DELAY );

		/* la tecla se pulsó */
		keys_data[index].state = STATE_BUTTON_RISING;

		break;

	case STATE_BUTTON_RISING:
		/* ENTRY */

		/* CHECK TRANSITION CONDITIONS */

		if( gpioRead( keys_config[index].tecla ) )
		{
			keys_data[index].state = STATE_BUTTON_UP;

			/* ACCION DEL EVENTO ! */
			buttonReleased( index );
		}
		else
		{
			keys_data[index].state = STATE_BUTTON_DOWN;
		}

		/* LEAVE */
		break;

	default:
		keys_ButtonError( index );
		break;
	}
}
#endif


/*=====[Implementations of private functions]================================*/

/* accion de el evento de tecla pulsada */
static void buttonPressed( uint32_t index )
{
	TickType_t current_tick_count = xTaskGetTickCount();

	taskENTER_CRITICAL();
	keys_data[index].time_down = current_tick_count;
	taskEXIT_CRITICAL();
}

/* accion de el evento de tecla liberada */
static void buttonReleased( uint32_t index )
{
	TickType_t current_tick_count = xTaskGetTickCount();

	taskENTER_CRITICAL();
	keys_data[index].time_up    = current_tick_count;
	keys_data[index].time_diff  = keys_data[index].time_up - keys_data[index].time_down;
	taskEXIT_CRITICAL();

	xSemaphoreGive( keys_data[index].pressed_signal ) ;
}

static void keys_ButtonError( uint32_t index )
{
	taskENTER_CRITICAL();
	keys_data[index].state = BUTTON_UP;
	taskEXIT_CRITICAL();
}

/*=====[Implementations of private functions]=================================*/
void task_tecla( void* taskParmPtr )
{
	uint32_t indice = (uint32_t) taskParmPtr;

	while( 1 )
	{
#if KEYS_USE_ISR==0
		keys_Update( indice );
		vTaskDelay( DEBOUNCE_TIME / portTICK_RATE_MS );
#else
		keys_Update_Isr( indice );
		vTaskDelay( DEBOUNCE_TIME / portTICK_RATE_MS );
#endif
	}
}

#if KEYS_USE_ISR==1
/**
   @brief   Inicializa las interrupciones asociadas al driver keys.c
			Se realiza sobre las cuatro teclas de la EDUCIAA
 */
void keys_isr_config( void )
{
	//Inicializamos las interrupciones (LPCopen)
	Chip_PININT_Init( LPC_GPIO_PIN_INT );

	//Inicializamos de cada evento de interrupcion (LPCopen)

	/* Machete:
    GLOBAL! extern pinInitGpioLpc4337_t gpioPinsInit[];
    Chip_SCU_GPIOIntPinSel( j,  gpioPinsInit[i].gpio.port, gpioPinsInit[i].gpio.pin );   // TECi
    Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH( j ) );                      // INTj (canal j -> hanlder GPIOj)       //Borra el pending de la IRQ
    Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH( j ) );                      // INTj //Selecciona activo por flanco
    Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH( j ) );                        // INTj //Selecciona activo por flanco descendente
    Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH( j ) );                       // INTj //Selecciona activo por flanco ascendente
	 */

	// TEC1 FALL
	Chip_SCU_GPIOIntPinSel( 0, 0, 4 ); 	//(Canal 0 a 7, Puerto GPIO, Pin GPIO)
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH0 ); //Se configura el canal para que se active por flanco
	Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH0 ); //Se configura para que el flanco sea el de bajada

	// TEC1 RISE
	Chip_SCU_GPIOIntPinSel( 1, 0, 4 );	//(Canal 0 a 7, Puerto GPIO, Pin GPIO)
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH1 ); //Se configura el canal para que se active por flanco
	Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH1 ); //En este caso el flanco es de subida


	// TEC2 FALL
	Chip_SCU_GPIOIntPinSel( 2, 0, 8 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH2 );
	Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH2 );

	// TEC1 RISE
	Chip_SCU_GPIOIntPinSel( 3, 0, 8 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH3 );
	Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH3 );

	// TEC3 FALL
	Chip_SCU_GPIOIntPinSel( 4, 0, 9 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH4 );
	Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH4 );

	// TEC1 RISE
	Chip_SCU_GPIOIntPinSel( 5, 0, 9 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH5 );
	Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH5 );

	// TEC4 FALL
	Chip_SCU_GPIOIntPinSel( 6, 1, 9 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH6 );
	Chip_PININT_EnableIntLow( LPC_GPIO_PIN_INT, PININTCH6 );

	// TEC4 RISE
	Chip_SCU_GPIOIntPinSel( 7, 1, 9 );
	Chip_PININT_SetPinModeEdge( LPC_GPIO_PIN_INT, PININTCH7 );
	Chip_PININT_EnableIntHigh( LPC_GPIO_PIN_INT, PININTCH7 );



	//Una vez que se han configurado los eventos para cada canal de interrupcion
	//Se activan las interrupciones para que comiencen a llamar al handler
	NVIC_SetPriority( PIN_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT0_IRQn );
	NVIC_SetPriority( PIN_INT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT1_IRQn );


	NVIC_SetPriority( PIN_INT2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT2_IRQn );
	NVIC_SetPriority( PIN_INT3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT3_IRQn );
	NVIC_SetPriority( PIN_INT4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT4_IRQn );
	NVIC_SetPriority( PIN_INT5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT5_IRQn );
	NVIC_SetPriority( PIN_INT6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT6_IRQn );
	NVIC_SetPriority( PIN_INT7_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
	NVIC_EnableIRQ( PIN_INT7_IRQn );

}

/**
   @brief handler de evento de tecla pulsada
   @param index
 */
void keys_isr_fall( uint32_t index )
{
	UBaseType_t uxSavedInterruptStatus;

	/* esta operacion debe realizarse en zona critica. Recordar que el objeto global puede estar leyendose
       o escribiendose en otro contexto  */
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	keys_data[index].time_down = xTaskGetTickCountFromISR();
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );

}

/**
   @brief handler de evento de tecla liberada
   @param index
 */
void keys_isr_rise( uint32_t index )
{
	UBaseType_t uxSavedInterruptStatus;

	/* esta operacion debe realizarse en zona critica. Recordar que el objeto global puede estar leyendose
       o escribiendose en otro contexto  */
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	keys_data[index].time_up = xTaskGetTickCountFromISR();
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}

void GPIO0_IRQHandler( void )   //asociado a tec1 FALL
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable

	if ( Chip_PININT_GetFallStates( LPC_GPIO_PIN_INT ) & PININTCH0 ) //Verificamos que la interrupciÃ³n es la esperada
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH0 ); //Borramos el flag de interrupciÃ³n

		keys_isr_fall( TEC1_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC1_INDEX].isr_signal, &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO1_IRQHandler( void )  //asociado a tec1 RISE
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if ( Chip_PININT_GetRiseStates( LPC_GPIO_PIN_INT ) & PININTCH1 )
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH1 );

		keys_isr_rise( TEC1_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC1_INDEX].isr_signal, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



void GPIO2_IRQHandler( void )	//asociado a tec2 FALL
{
	UBaseType_t uxSavedInterruptStatus;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable

	if ( Chip_PININT_GetFallStates( LPC_GPIO_PIN_INT ) & PININTCH2 ) //Verificamos que la interrupciÃ³n es la esperada
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH2 ); //Borramos el flag de interrupciÃ³n

		keys_isr_fall( TEC2_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC2_INDEX].isr_signal, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



void GPIO3_IRQHandler( void )	//asociado a tec2 RISE
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if ( Chip_PININT_GetRiseStates( LPC_GPIO_PIN_INT ) & PININTCH3 )
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH3 );
		//codigo a ejecutar si ocurriÃ³ la interrupciÃ³n

		keys_isr_rise( TEC2_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC2_INDEX].isr_signal, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO4_IRQHandler( void )	////asociado a tec3 FALL
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable


	if ( Chip_PININT_GetFallStates( LPC_GPIO_PIN_INT ) & PININTCH4 ) //Verificamos que la interrupciÃ³n es la esperada
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH4 ); //Borramos el flag de interrupciÃ³n

		keys_isr_fall( TEC3_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC3_INDEX].isr_signal, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO5_IRQHandler( void )	//asociado a tec3 RISE
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if ( Chip_PININT_GetRiseStates( LPC_GPIO_PIN_INT ) & PININTCH5 )
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH5 );

		keys_isr_rise( TEC3_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC3_INDEX].isr_signal, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO6_IRQHandler( void )	//asociado a tec4 FALL
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable

	if ( Chip_PININT_GetFallStates( LPC_GPIO_PIN_INT ) & PININTCH6 ) //Verificamos que la interrupciÃ³n es la esperada
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH6 ); //Borramos el flag de interrupciÃ³n

		keys_isr_fall( TEC4_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC4_INDEX].isr_signal, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO7_IRQHandler( void )	//asociado a tec4 RISE
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if ( Chip_PININT_GetRiseStates( LPC_GPIO_PIN_INT ) & PININTCH7 )
	{
		Chip_PININT_ClearIntStatus( LPC_GPIO_PIN_INT, PININTCH7 );

		keys_isr_rise( TEC4_INDEX );

		xSemaphoreGiveFromISR( keys_data[TEC4_INDEX].isr_signal, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


#endif
