/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pic_spi.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "pic_spi.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

PIC_SPI_DATA pic_spiData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
/*** ADD ***/
void callbackTimerWrite( uintptr_t context, uint32_t currTick )
{
    pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_WRITE;
    return;
}

void callbackTimerRead( uintptr_t context, uint32_t currTick )
{
    pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_READ;
    return;
}

void callbackSpiWriteMaster( DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context )
{
    pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_READTIMER;
    return;
}

void callbackSpiReadMaster( DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context )
{
    pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_FINISH;
    return;
}

void callbackSpiWriteSlave( DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context )
{
    pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_FINISH;
    return;
}

void callbackSpiReadSlave( DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context ){
    pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_WRITE;
    return;
}
/*** ADD ***/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
static void PIC_SPI_Master_Tasks(void)
{
    switch(pic_spiData.stateMaster)
    {
        case PIC_SPI_MASTER_STATE_INIT:
        {
            pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_WRITETIMER;
            break;
        }
        case PIC_SPI_MASTER_STATE_WRITETIMER:
        {
            pic_spiData.timerHandle = SYS_TMR_ObjectCreate(2, NULL, callbackTimerWrite, SYS_TMR_FLAG_SINGLE);
            if( pic_spiData.timerHandle != SYS_TMR_HANDLE_INVALID ){
                pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_WRITETIMERWAIT;
            }
            break;
        }
        case PIC_SPI_MASTER_STATE_WRITE:
        {
            SS1Off();
            
            pic_spiData.masterWriteData = 0x33;
            
            pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_WRITEWAIT;
            
            DRV_SPI_BufferAddWrite2(
                    pic_spiData.spiHandleMaster,
                    &pic_spiData.masterWriteData,
                    1,
                    callbackSpiWriteMaster,
                    NULL,
                    &pic_spiData.spiBufferHandleMaster
                    );
            if(pic_spiData.spiBufferHandleMaster == DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_WRITE;
            }
            break;
        }
        case PIC_SPI_MASTER_STATE_READTIMER:
        {
            SYS_TMR_ObjectReload(pic_spiData.timerHandle, 1, NULL, callbackTimerRead);
            pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_READTIMERWAIT;
            break;
        }
        case PIC_SPI_MASTER_STATE_READ:
        {
            pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_READWAIT;
            
            DRV_SPI_BufferAddRead2(
                    pic_spiData.spiHandleMaster,
                    &pic_spiData.masterReadData,
                    1,
                    callbackSpiReadMaster,
                    NULL,
                    &pic_spiData.spiBufferHandleMaster
                    );
            if(pic_spiData.spiBufferHandleMaster == DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_READWAIT;
            }
            break;
        }
        case PIC_SPI_MASTER_STATE_FINISH:
        {
            SS1On();
            
            break;
        }
        default:
            break;
    }
    return;
}

static void PIC_SPI_Slave_Tasks(void)
{
    switch(pic_spiData.stateSlave)
    {
        case PIC_SPI_SLAVE_STATE_INIT:
        {
            pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_READ;
            break;
        }
        case PIC_SPI_SLAVE_STATE_READ:
        {
            pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_READWAIT;
            
            DRV_SPI_BufferAddRead2(
                    pic_spiData.spiHandleSlave,
                    &pic_spiData.slaveReadData,
                    1,
                    callbackSpiReadSlave,
                    NULL,
                    &pic_spiData.spiBufferHandleSlave
                    );
            if(pic_spiData.spiBufferHandleSlave == DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_READ;
            }
        }
        case PIC_SPI_SLAVE_STATE_WRITE:
        {
            pic_spiData.slaveWriteData = 0x66;
            
            pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_WRITEWAIT;
                
            DRV_SPI_BufferAddWrite2(
                pic_spiData.spiHandleSlave,
                &pic_spiData.slaveWriteData,
                1,
                callbackSpiWriteSlave,
                NULL,
                &pic_spiData.spiBufferHandleSlave
            );
            if(pic_spiData.spiBufferHandleSlave == DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_WRITE;
            }
            break;
        }
        case PIC_SPI_SLAVE_STATE_FINISH:
        {
            break;
        }
    }
    return;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIC_SPI_Initialize ( void )

  Remarks:
    See prototype in pic_spi.h.
 */

void PIC_SPI_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pic_spiData.state = PIC_SPI_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    /*** ADD ***/
    pic_spiData.spiHandleMaster = DRV_HANDLE_INVALID;
    pic_spiData.spiHandleSlave = DRV_HANDLE_INVALID;
    pic_spiData.stateMaster = PIC_SPI_MASTER_STATE_INIT;
    pic_spiData.stateSlave = PIC_SPI_SLAVE_STATE_INIT;
    pic_spiData.spiBufferHandleMaster = DRV_SPI_BUFFER_HANDLE_INVALID;
    pic_spiData.spiBufferHandleSlave = DRV_SPI_BUFFER_HANDLE_INVALID;
    pic_spiData.timerHandle = SYS_TMR_HANDLE_INVALID;
    pic_spiData.masterWriteData = 0x00;
    pic_spiData.masterReadData = 0x00;
    pic_spiData.slaveWriteData = 0x00;
    pic_spiData.slaveReadData = 0x00;
    /*** ADD ***/
}


/******************************************************************************
  Function:
    void PIC_SPI_Tasks ( void )

  Remarks:
    See prototype in pic_spi.h.
 */

void PIC_SPI_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( pic_spiData.state )
    {
        /* Application's initial state. */
        case PIC_SPI_STATE_INIT:
        {
            bool appInitialized = true;
            
            /*** ADD ***/
            if (pic_spiData.spiHandleMaster == DRV_HANDLE_INVALID)
            {
                pic_spiData.spiHandleMaster = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING );
                appInitialized &= ( DRV_HANDLE_INVALID != pic_spiData.spiHandleMaster );
            }
       
            if (pic_spiData.spiHandleSlave == DRV_HANDLE_INVALID)
            {
                pic_spiData.spiHandleSlave = DRV_SPI_Open( DRV_SPI_INDEX_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING );
                appInitialized &= ( DRV_HANDLE_INVALID != pic_spiData.spiHandleSlave );
            }
            /*** ADD ***/
            
            if (appInitialized)
            {
                /*** ADD ***/
                SS1On();
                /*** ADD ***/
            
                pic_spiData.state = PIC_SPI_STATE_SERVICE_TASKS;
            }
            break;
        }

        case PIC_SPI_STATE_SERVICE_TASKS:
        {
            /*** ADD ***/
            PIC_SPI_Master_Tasks();
            PIC_SPI_Slave_Tasks();
            /*** ADD ***/
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
