
/*******************************************************************************
 *
 *  
 ******************************************************************************/


/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

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

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

//Standard C libraries
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

//PIC32 system libraries
#include "system_config.h"
#include "system_definitions.h"

//User libraries

#include "ringbuffer.h"
#include "ArduCAM/arducam.h"



// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

void msgStatus(char *(msg));

// *****************************************************************************
// *****************************************************************************
// Section: Constant Data
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* copied from Harmony example - think that this makes sure that 32 bytes of data
                                    properly align with the filedata[] array*/
#ifdef DRV_SDHC_USE_DMA
#define DATA_BUFFER_ALIGN             __attribute__((coherent, aligned(32)))
#else
#define DATA_BUFFER_ALIGN             __attribute__((aligned(32)))
#endif

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

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
	APP_STATE_INIT=0,
	
    // Disk, arducam and wiznet initialisation
    APP_STATE_SETUP_ARDUCAM,
    APP_STATE_MOUNT_DISK,
    APP_STATE_SET_CURRENT_DRIVE,
            
    //Currently unused.        
    APP_STATE_CLOSE_FILE,
    APP_STATE_UNMOUNT_DISK,
            
    APP_STATE_NORMAL,
            
    // Arducam state machine        
    APP_STATE_TAKE_PHOTO,
    APP_STATE_PHOTO_READY,
    APP_STATE_OPEN_FILE,
    APP_STATE_SAVE_PHOTO,        
     
    // Uart state machines
    APP_STATE_ANALYSE_RX_BUFFER,
    APP_STATE_TX_BUFFER_SEND,
            
    // LCD state machine
    APP_STATE_WRITE_LCD_MSG,
      
            
    APP_STATE_ERROR,
	

} APP_STATES;


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
    APP_STATES state;

    /*Usart stuff - ring buffers for transmit and receive*/
    DRV_HANDLE   usartHandle;
    ringBuffer_t rxRingBuffer;
    ringBuffer_t txRingBuffer;
    bool         lcdInitialised;

    /* LCD stuff - write buffer, cursor position, etc */
    ringBuffer_t LCDRingBuffer;
    int          xcursorLCD;
    int          ycursorLCD;
   
    /* SDCard stuff - fileHandle(n):    Handle for each file you want to work on
     *              - fileData:         buffer for data (see above for DATA_BUFFER_ALIGN
     *              - nBytes:           number of bytes read/written */
    int                j;
    SYS_FS_HANDLE      logHandle;
    SYS_FS_HANDLE      jpgHandle1;
    SYS_FS_HANDLE      jpgHandle2;
    uint8_t           fileData[256] DATA_BUFFER_ALIGN;
    
    // arducam stuff
    DRV_HANDLE         arducamI2CHandle;
    bool                camSetup;
    bool                isI2CReceived;
    bool                isI2CSent;
    uint8_t             vid;
    uint8_t             pid; 
    
    bool                takePhoto;
    bool                pollPhoto;
    bool                photoReady;
    bool                fileOpened;
    
} APP_DATA;

APP_DATA appData;



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
    void APP_Initialize ( void )

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
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

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
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

