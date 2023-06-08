#ifndef PORTMACRO_H
#define PORTMACRO_H
#include <stdint.h>
#include <ppc_ghs.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Type definitions. */
#define portCHAR       char
#define portFLOAT      float
#define portDOUBLE     double
#define portLONG       long
#define portSHORT      short
#define portSTACK_TYPE uint32_t
#define portBASE_TYPE  int

typedef portSTACK_TYPE StackType_t;
typedef portBASE_TYPE BaseType_t;
typedef unsigned portBASE_TYPE UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
    typedef uint16_t TickType_t;
    #define portMAX_DELAY ( ( TickType_t ) UINT16_MAX )
#else
    typedef uint32_t TickType_t;
    #define portMAX_DELAY ( ( TickType_t ) UINT32_MAX )
#endif

/* This port uses the critical nesting count from the TCB rather than
maintaining a separate value and then saving this value in the task stack. */
#define portCRITICAL_NESTING_IN_TCB 1

/* Interrupt control macros. */
#define portDISABLE_INTERRUPTS() __DI()
#define portENABLE_INTERRUPTS()  __EI()

/* Critical section macros. */
void vTaskEnterCritical( void );
void vTaskExitCritical( void );
#define portENTER_CRITICAL() vTaskEnterCritical()
#define portEXIT_CRITICAL()  vTaskExitCritical()

/* Task utilities. */
#define portYIELD() asm( "SC \n\t NOP" )
#define portYIELD_FROM_ISR( xHigherPriorityTaskWoken ) if( xHigherPriorityTaskWoken != pdFALSE ) vTaskSwitchContext()

/* Hardware specifics. */
#define portBYTE_ALIGNMENT 16
#define portSTACK_GROWTH   ( -1 )
#define portTICK_PERIOD_MS ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portNOP()          asm( "nop" )

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#ifdef __cplusplus
}
#endif

#endif // PORTMACRO_H
