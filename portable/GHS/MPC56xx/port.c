#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#ifdef portDEBUG
#include <alloca.h>
#endif
#include <ppc_ghs.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file.
 */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "semphr.h"

extern const char __ghsbegin_sdata2[];
extern char __ghsbegin_sdabase[];
#ifdef portDEBUG
extern StackType_t __ghsbegin_stack[];
#endif
extern int FreeRTOS_errno;

typedef void ( * ExceptionHandler_t ) ( void );

#ifdef portDEBUG
StackType_t * pxCurrentStack;
#endif

static BaseType_t xShuttingDown;
static jmp_buf xEndScheduler;
static ExceptionHandler_t pxOldSystemCallHandler, pxOldDecrementerHandler;
#if configSUPPORT_STATIC_ALLOCATION
static StaticSemaphore_t xPortGHSLockMutexBuffer, xPortFileIOLockMutexBuffer;
#endif // configSUPPORT_STATIC_ALLOCATION
static SemaphoreHandle_t xPortGHSLock, xPortFileIOLock;
#ifdef portDEBUG
# ifdef __ghs_pid
/* Using this variable instead of directly referencing __ghsbegin_stack
   gives the linker a chance to fix up this address.
   This is important if, for example, we link with PID but the stack is
   absoultely located in the link map. */
static StackType_t* begin_stack_addr = __ghsbegin_stack;
# else
#define begin_stack_addr __ghsbegin_stack
# endif // __ghs_pid
#endif // portDEBUG

/*----------------------------------------------------------------------------*/
/* Default hook functions.
 */

#if configUSE_IDLE_HOOK

#pragma weak vApplicationIdleHook
void vApplicationIdleHook( void )
{
    configASSERT( 0 );
}

#endif // configUSE_IDLE_HOOK

#if configUSE_TICK_HOOK

#pragma weak vApplicationTickHook
void vApplicationTickHook( void )
{
    configASSERT( 0 );
}

#endif // configUSE_TICK_HOOK

#if configUSE_MALLOC_FAILED_HOOK

#pragma weak vApplicationMallocFailedHook
void vApplicationMallocFailedHook( void )
{
    configASSERT( 0 );
}

#endif // configUSE_MALLOC_FAILED_HOOK

#if configUSE_TIMERS && configUSE_DAEMON_TASK_STARTUP_HOOK

#pragma weak vApplicationDaemonTaskStartupHook
void vApplicationDaemonTaskStartupHook( void )
{
    configASSERT( 0 );
}

#endif // configUSE_TIMERS && configUSE_DAEMON_TASK_STARTUP_HOOK

#if configCHECK_FOR_STACK_OVERFLOW

#pragma weak vApplicationStackOverflowHook
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    configASSERT( 0 );
}

#endif // configCHECK_FOR_STACK_OVERFLOW

/*----------------------------------------------------------------------------*/
/* Static memory allocations.
 */

#if configSUPPORT_STATIC_ALLOCATION

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide
 * an implementation of vApplicationGetIdleTaskMemory() to provide the memory
 * that is used by the Idle task.
 */
#pragma weak vApplicationGetIdleTaskMemory
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits.
     */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored.
     */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes.
     */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

# if configUSE_TIMERS

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so
 * the application must provide an implementation of
 * vApplicationGetTimerTaskMemory() to provide the memory that is used by the
 * Timer service task.
 */
#pragma weak vApplicationGetTimerTaskMemory
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits.
     */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored.
     */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes.
     */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

# endif // configUSE_TIMERS
#endif // configSUPPORT_STATIC_ALLOCATION

/*----------------------------------------------------------------------------*/
/* Dynamic memory allocations.
 */

#if configSUPPORT_DYNAMIC_ALLOCATION

void * pvPortMalloc( size_t xWantedSize )
{
    void * pvReturn;

    pvReturn = malloc( xWantedSize );
    traceMALLOC( pvReturn, xWantedSize );

# if configUSE_MALLOC_FAILED_HOOK
    if( pvReturn == NULL )
    {
#  ifdef portDEBUG
        static volatile size_t xRequestedSize;
        xRequestedSize = xWantedSize;
#  endif // portDEBUG
        vApplicationMallocFailedHook();
    }
# endif // configUSE_MALLOC_FAILED_HOOK

    return pvReturn;
}

void vPortFree( void * pv )
{
    if( pv )
    {
        free( pv );
        traceFREE( pv, 0 );
    }
}

# if configAPPLICATION_ALLOCATED_HEAP

uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

# endif // configAPPLICATION_ALLOCATED_HEAP

# if configSTACK_ALLOCATION_FROM_SEPARATE_HEAP
#  error configSTACK_ALLOCATION_FROM_SEPARATE_HEAP not supported
# endif // configSTACK_ALLOCATION_FROM_SEPARATE_HEAP
#endif // configSUPPORT_DYNAMIC_ALLOCATION

/*----------------------------------------------------------------------------*/
/* Task stack initialization.
 */

/* Definitions to set the initial MSR of each task. */
#define portSIGNAL_PROCESSING_ENGINE_ENABLE ( 1U << 25 )
#define portCRITICAL_INTERRUPT_ENABLE       ( 1U << 17 )
#define portEXTERNAL_INTERRUPT_ENABLE       ( 1U << 15 )
#define portUSER_MODE                       ( 1U << 14 )
#define portFLOATING_POINT_ENABLE           ( 1U << 13 )
#define portMACHINE_CHECK_ENABLE            ( 1U << 12 )

#define portINITIAL_MSR ( portSIGNAL_PROCESSING_ENGINE_ENABLE | \
                          portEXTERNAL_INTERRUPT_ENABLE | \
                          portFLOATING_POINT_ENABLE | \
                          portMACHINE_CHECK_ENABLE )

/* The following must agree with portasm.ppc. */
#define portINTERRUPT_STACK_LENGTH 76
#define portINITIAL_STACK_LENGTH   ( portINTERRUPT_STACK_LENGTH + 4 )
#define portINITIAL_STACK_SIZE     ( portINITIAL_STACK_LENGTH * \
                                     sizeof( StackType_t ) )
#define portBACKCHAIN_OFFSET       0
#define portR31_OFFSET             2
#define portPC_OFFSET              68
#define portMSR_OFFSET             69
#define portLR_OFFSET              72

/*
 * Initialise the stack of a task to look exactly as if the task had been
 * interrupted.
 *
 * See the header file portable.h.
 */
#if __GlobalRegisters != 0
#error -globalreg support not implemented
#endif

StackType_t * pxPortInitialiseStack( StackType_t *pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void *pvParameters )
{
    pxTopOfStack -= portINITIAL_STACK_LENGTH;

    memset( pxTopOfStack, 0, portINITIAL_STACK_SIZE );

    /* Backchain */
    pxTopOfStack[ portBACKCHAIN_OFFSET ] =
        ( StackType_t ) ( pxTopOfStack + portINTERRUPT_STACK_LENGTH );

    /* Address of the read-write small data area in lower part of double R13 */
    pxTopOfStack[ portR31_OFFSET + 2 * ( 31 - 13 ) + 1 ] =
        ( StackType_t ) ( __ghsbegin_sdabase + 0x8000 );

    /* Parameters in lower part of double R3 */
    pxTopOfStack[ portR31_OFFSET + 2 * ( 31 - 3 ) + 1 ] =
        ( StackType_t ) pvParameters;

    /* Address of the read-only small data area in lower part of double R2 */
    pxTopOfStack[ portR31_OFFSET + 2 * ( 31 - 2 ) + 1 ] =
        ( StackType_t ) ( __ghsbegin_sdata2 + 0x8000 );

    /* PC */
    pxTopOfStack[ portPC_OFFSET ] = ( StackType_t ) pxCode;

    /* MSR */
    pxTopOfStack[ portMSR_OFFSET ] = portINITIAL_MSR;

    /* LR */
    pxTopOfStack[ portLR_OFFSET ] = ( StackType_t ) vPortEndScheduler;

    /* Place a known value at the bottom of the stack for debugging */
    pxTopOfStack[ portINITIAL_STACK_LENGTH - 1 ] = 0xDEADBEEF;

    return pxTopOfStack;
}

/*----------------------------------------------------------------------------*/
/* System startup and shutdown.
 */

/* Special Purpose Registers */
#define portDEC                22   // Decrementer
#define portDECAR              54   // Decrementer Auto-Reload
#define portIVPR               63   // Interrupt Vector Prefix Register
#define portTSR                336  // Timer Status Register
#define portTCR                340  // Timer Control Register
#define portVECTOR_SYSTEM_CALL 408  // System Call Interrupt Offset
#define portVECTOR_DECREMENTER 410  // Decrementer Interrupt Offset
#define portHID0               1008 // HW Implementation Dependent Register 0

/* Special Purpose Register Values */
#define portTSR_DIS   ( 1U << 27 ) // Decrementer Interrupt Status
#define portTCR_DIE   ( 1U << 26 ) // Decrementer Interrupt Enable
#define portTCR_ARE   ( 1U << 22 ) // Auto-Reload Enable
#define portHID0_TBEN ( 1U << 14 ) // Time Base Enable

/* Register exception handler & return previously registered handler. */
static inline ExceptionHandler_t prvPortRegisterExceptionHandler(
        unsigned ulVector, ExceptionHandler_t pxHandler )
{
    unsigned uxIVPR = __MFSPR( portIVPR );
    unsigned uxOldHandlerOffset = __MFSPR( ulVector );

    configASSERT( ( ( unsigned ) pxHandler & 0xffff0000 ) == uxIVPR );

    __MTSPR( ulVector, ( unsigned ) pxHandler & 0x0000ffff );
    return ( ExceptionHandler_t ) ( uxIVPR | uxOldHandlerOffset );
}

/* Function to start the scheduler running by starting the highest priority task
 * that has thus far been created.
 */
extern void vPortStartFirstTask( void );

/* Function to handle System Call. */
extern void vPortSystemCallHandler( void );

/* Function to handle tick interrupt. */
extern void vPortTickISR( void );

static void prvPortSetupTimerInterrupt( void )
{
    const unsigned uxDECValue = configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ;
    __MTSPR( portDECAR, uxDECValue );
    __MTSPR( portDEC, uxDECValue );
    __MTSPR( portTSR, portTSR_DIS );
    __MTSPR( portTCR, portTCR_DIE | portTCR_ARE );
    __MTSPR( portHID0, portHID0_TBEN );
}

__attribute__((stackcheck(0)))
static void prvPortCleanupTimerInterrupt( void )
{
    __MTSPR( portTCR, 0 );
    __MTSPR( portHID0, 0 );
}

static void prvPortSetup( void )
{
    prvPortSetupTimerInterrupt();
    pxOldSystemCallHandler = prvPortRegisterExceptionHandler(
        portVECTOR_SYSTEM_CALL, vPortSystemCallHandler );
    pxOldDecrementerHandler = prvPortRegisterExceptionHandler(
        portVECTOR_DECREMENTER, vPortTickISR );
}

__attribute__((stackcheck(0)))
static void prvPortCleanup( void )
{
    if( xShuttingDown )
        return;
    xShuttingDown = pdTRUE;

#ifdef portDEBUG
    pxCurrentStack = NULL;
#endif
    prvPortCleanupTimerInterrupt();
    if( pxOldSystemCallHandler )
        prvPortRegisterExceptionHandler(
                portVECTOR_SYSTEM_CALL, pxOldSystemCallHandler );
    if( pxOldDecrementerHandler )
        prvPortRegisterExceptionHandler(
                portVECTOR_DECREMENTER, pxOldDecrementerHandler );
}

BaseType_t xPortStartScheduler( void )
{
    if( setjmp(xEndScheduler) == 0 )
    {
        prvPortSetup();
        vPortStartFirstTask();

        /* Should not get here as the tasks are now running! */
        return pdTRUE;
    } else {
        prvPortCleanup();
        return pdFALSE;
    }
}

__attribute__((stackcheck(0)))
void vPortEndScheduler( void )
{
    longjmp( xEndScheduler, 1 );
}

/*
 * The following functions are wrappers to _assert_ & _Exit. Add the following
 * to the program's .gpj file:

	-lnk='-wrap _assert_'
	-lnk='-wrap _Exit'

*/

extern void __real__assert_( const char *, const char *, int, const char * );

__attribute__((stackcheck(0)))
void __wrap__assert_(
    const char * msg, const char * file, int line, const char * func )
{
    static BaseType_t xAsserted;

    if( xAsserted )
        return;
    xAsserted = pdTRUE;

    __DI();
    prvPortCleanup();
    __real__assert_( msg, file, line, func );
}

extern void __real__Exit( int status );

__attribute__((stackcheck(0)))
void __wrap__Exit( int status )
{
    __DI();
    prvPortCleanup();
    __real__Exit( status );
}

/*----------------------------------------------------------------------------*/
/* libsys
*/

/* ----- errno -----
 *
 * Unfortunately, FreeRTOS's and GHS's implementations of thread local storage
 * are basically incompatible. GHS's implementation depends on being able to
 * determine the address of the thread's local storage area, whereas FreeRTOS,
 * although it allocates space in the Task Control Block for local storage, only
 * allows indirect reading and writing of the area, not taking its address.
 *
 * Therefore, this errno implementation for FreeRTOS cannot use the normal GHS
 * method. Rather, it will use FreeRTOS' configUSE_POSIX_ERRNO configuration
 * option which causes FreeRTOS to declare a single, global errno variable
 * (namely, FreeRTOS_errno), and then saves & restores it (to/from TCB's) when
 * context switching.
 */

int *__gh_errno_ptr( void )
{
    return &FreeRTOS_errno;
}

void __gh_set_errno( int err )
{
    int *p = __gh_errno_ptr();
    *p = err;
}

int  __gh_get_errno( void )
{
    int *p = __gh_errno_ptr();
    return *p;
}

/* ----- Global lock ----- */

/* Acquire global lock. Blocks until the lock becomes available. */
void __ghsLock( void )
{
    if( xShuttingDown || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING )
        return;

    xSemaphoreTakeRecursive( xPortGHSLock, portMAX_DELAY );
}

/* Release global lock. */
void __ghsUnlock( void )
{
    if( xShuttingDown || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING )
        return;

    xSemaphoreGiveRecursive( xPortGHSLock );
}

/* A callback to initialize the lock data structure before it is used. */
void __gh_lock_init( void )
{
    if( xShuttingDown )
        return;

#if configSUPPORT_STATIC_ALLOCATION
    xPortGHSLock = xSemaphoreCreateRecursiveMutexStatic(
        &xPortGHSLockMutexBuffer );
#else
    /* Note that this will call malloc, which will call __ghsLock (above.)
     * This is ok because this is being done before the scheduler is started,
     * so __ghsLock will simply return. Same goes for __ghsUnlock.
     */
    xPortGHSLock = xSemaphoreCreateRecursiveMutex();
#endif // configSUPPORT_STATIC_ALLOCATION
    configASSERT( xPortGHSLock );
    vQueueAddToRegistry( ( QueueHandle_t ) xPortGHSLock, "GHSLock" );
}

/* ----- File Locks -----
 *
 * These routines can be customized to implement per-file locks to allow
 * thread-safe I/O.
 *
 * Initial implementation uses a unique mutex from ghsLock, but only one for all
 * files.
 *
 * Default implementations of the __ghs_fxxx functions are defined in
 * libsys/ind_thrd.c. Normally, the implementations below will replace them via
 * the "pragma weak" lines below, as long as the FreeRTOS library this module is
 * part of gets linked first. Unfortunately, ind_thrd.c also defines other
 * symbols (e.g., for thread local storage), so ind_thrd might get pulled in by
 * the linker (e.g., for C++ programs using exceptions), which will cause its
 * implementations of these functions to be used instead. If this happens, use
 * the linker's -wrap option for each of the __ghs_fxxx symbols. E.g., put the
 * following in the program's .gpj file:

	-lnk='-wrap __ghs_flock_file'
	-lnk='-wrap __ghs_funlock_file'
	-lnk='-wrap __ghs_ftrylock_file'
	-lnk='-wrap __ghs_flock_create'
	-lnk='-wrap __ghs_flock_destroy'

 * This will cause the linker to replace references to __ghs_fxxx with calls to
 * __wrap___ghs_fxxx, which will cause the implementations below to be used.
*/

#pragma weak __ghs_flock_file = __wrap___ghs_flock_file
#pragma weak __ghs_funlock_file = __wrap___ghs_funlock_file
#pragma weak __ghs_ftrylock_file = __wrap___ghs_ftrylock_file
#pragma weak __ghs_flock_create = __wrap___ghs_flock_create
#pragma weak __ghs_flock_destroy = __wrap___ghs_flock_destroy

/* Acquire lock for FILE using specified lock. */
void __wrap___ghs_flock_file( void *addr )
{
    if( xShuttingDown || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING )
        return;

    xSemaphoreTakeRecursive( ( SemaphoreHandle_t ) addr, portMAX_DELAY );
}

/* Release lock for FILE using specified lock. */
void __wrap___ghs_funlock_file(void *addr)
{
    if( xShuttingDown || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING )
        return;

    xSemaphoreGiveRecursive( ( SemaphoreHandle_t ) addr );
}

/* Non blocking acquire lock for FILE using specified lock.  May return -1 if
 * this is not implemented. Returns 0 on success and nonzero otherwise.
 */
int __wrap___ghs_ftrylock_file(void *addr)
{
    if( xShuttingDown )
        return 3;
    if( xTaskGetSchedulerState() != taskSCHEDULER_RUNNING )
        return 2;
    if( xSemaphoreTakeRecursive( ( SemaphoreHandle_t ) addr, 0 ) != pdTRUE )
        return 1;
    return 0;
}

/* Callbacks to initialize local lock data structures before they are used. */
void __wrap___ghs_flock_create(void **addr)
{
    if( xShuttingDown )
        return;

    if( !xPortFileIOLock )
    {
#if configSUPPORT_STATIC_ALLOCATION
        xPortFileIOLock = xSemaphoreCreateRecursiveMutexStatic(
            &xPortFileIOLockMutexBuffer );
#else
        xPortFileIOLock = xSemaphoreCreateRecursiveMutex();
#endif // configSUPPORT_STATIC_ALLOCATION
        configASSERT( xPortFileIOLock );
        vQueueAddToRegistry( ( QueueHandle_t ) xPortFileIOLock, "FileIOLock" );
    }
    *addr = xPortFileIOLock;
}

void __wrap___ghs_flock_destroy( void *addr ) {}

/* ----- Stack Checking ----- */

#ifdef portDEBUG

#pragma ghs max_instances 2
void __stkchk( void )
{
    static int did_error = 0;
#ifdef __GHS_TARGET_IMPLEMENTS_ALLOCA
    StackType_t *sp = __get_stack_pointer();
#else
    char loc;
    StackType_t *sp = ( void * )&loc;
#endif /* __GHS_TARGET_IMPLEMENTS_ALLOCA */

    if( pxCurrentStack )
    {
        if ( pxCurrentStack < sp )
            return;
    } else {
        if( begin_stack_addr < sp )
            return;
    }
    if( did_error )
        return;
    did_error = 1;
    exit(1);
}

#endif // portDEBUG
