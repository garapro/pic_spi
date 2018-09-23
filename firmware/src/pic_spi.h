/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pic_spi.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _PIC_SPI_H
#define _PIC_SPI_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	PIC_SPI_STATE_INIT=0,
	PIC_SPI_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} PIC_SPI_STATES;

/*** ADD ***/
// SPI MASTER 状態
typedef enum{
    PIC_SPI_MASTER_STATE_INIT=0,                // 初期
    PIC_SPI_MASTER_STATE_WRITETIMER,            // 書き込みタイマー
    PIC_SPI_MASTER_STATE_WRITETIMERWAIT,        // 書き込みタイムアウト待ち
    PIC_SPI_MASTER_STATE_WRITE,                 // 書き込み
    PIC_SPI_MASTER_STATE_WRITEWAIT,             // 書き込み待ち
    PIC_SPI_MASTER_STATE_READTIMER,             // 読み込みタイマー
    PIC_SPI_MASTER_STATE_READTIMERWAIT,         // 読み込みタイムアウト待ち
    PIC_SPI_MASTER_STATE_READ,                  // 読み込み
    PIC_SPI_MASTER_STATE_READWAIT,              // 読み込み待ち
    PIC_SPI_MASTER_STATE_FINISH,                // 完了
} PIC_SPI_MASTER_STATES;

// SPI SLAVE 状態
typedef enum
{
    PIC_SPI_SLAVE_STATE_INIT=0,                 // 初期
    PIC_SPI_SLAVE_STATE_READ,                   // 読み込み
    PIC_SPI_SLAVE_STATE_READWAIT,               // 読み込み待ち
    PIC_SPI_SLAVE_STATE_WRITE,                  // 書き込み
    PIC_SPI_SLAVE_STATE_WRITEWAIT,              // 書き込み待ち
    PIC_SPI_SLAVE_STATE_FINISH,                 // 完了
} PIC_SPI_SLAVE_STATES;
/*** ADD ***/

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    PIC_SPI_STATES state;

    /* TODO: Define any additional data used by the application. */
    /*** ADD ***/
    PIC_SPI_MASTER_STATES stateMaster;                          // SPI Master 状態
    PIC_SPI_SLAVE_STATES stateSlave;                            // SPI Slave 状態
    
    DRV_HANDLE spiHandleMaster;                                 // SPI Master ハンドル
    DRV_HANDLE spiHandleSlave;                                  // SPI Slave ハンドル
    
    DRV_SPI_BUFFER_HANDLE spiBufferHandleMaster;                // SPI Master バッファハンドル
    DRV_SPI_BUFFER_HANDLE spiBufferHandleSlave;                 // SPI Slave バッファハンドル
    
    SYS_TMR_HANDLE timerHandle;                                 // タイマーハンドル
    
    uint8_t masterWriteData;                                    // SPI Master 書き込みデータ
    uint8_t masterReadData;                                     // SPI Master 読み込みデータ
    uint8_t slaveWriteData;                                     // SPI Slave 書き込みデータ
    uint8_t slaveReadData;                                      // SPI Slave 読み込みデータ
    /*** ADD ***/
} PIC_SPI_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIC_SPI_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PIC_SPI_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PIC_SPI_Initialize ( void );


/*******************************************************************************
  Function:
    void PIC_SPI_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PIC_SPI_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void PIC_SPI_Tasks( void );


#endif /* _PIC_SPI_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

