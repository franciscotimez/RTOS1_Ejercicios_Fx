/*
 * keys.h
 *
 *  Created on: 7 oct. 2020
 *      Author: francisco
 */

#ifndef KEYS_H_
#define KEYS_H_

#include "FreeRTOS.h"
#include "semphr.h"

/* public macros ================================================================= */
#define KEYS_INVALID_TIME   -1

#define KEYS_USE_ISR        1

#define TEC1_INDEX  0
#define TEC2_INDEX  1
#define TEC3_INDEX  2
#define TEC4_INDEX  3


/* types ================================================================= */
typedef enum
{
    STATE_BUTTON_UP,
    STATE_BUTTON_DOWN,
    STATE_BUTTON_FALLING,
    STATE_BUTTON_RISING
} keys_ButtonState_t;

typedef struct
{
    gpioMap_t tecla;			//config
} t_key_config;

typedef struct
{
    keys_ButtonState_t state;   //variables
    TickType_t time_down;		//timestamp of the last High to Low transition of the key
    TickType_t time_up;		    //timestamp of the last Low to High transition of the key
    TickType_t time_diff;	    //variables

#if KEYS_USE_ISR==1
    SemaphoreHandle_t isr_signal;   //almacenara el handle del semaforo creado para una cierta tecla
#endif

    SemaphoreHandle_t pressed_signal;

} t_key_data;

/* methods ================================================================= */
void keys_Init( void );
TickType_t get_diff();
void clear_diff();
int key_pressed( uint32_t index );

#endif /* PDM_ANTIRREBOTE_MEF_INC_DEBOUNCE_H_ */
