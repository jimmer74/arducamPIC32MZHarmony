/*******************************************************************************
 *
 * 
 * 
 ******************************************************************************/



/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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
Software only when embedded on a Microchip mi
 * crocontroller or digital signal
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

#include "app.h"
//User libraries
#include "delay.h"



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

extern APP_DATA appData;



// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

//initialise some variables we're going to need

    // camera stuff
    appData.arducamSPIHandle = DRV_HANDLE_INVALID;
    appData.arducamI2CHandle = DRV_HANDLE_INVALID;
    
    //initialisation flags
    appData.isTmr    = false;
    appData.isI2CReceived  = false;
    appData.isI2CSent = false;
    appData.spiDone    = false;
    appData.camSetup   = false;
    
    appData.takePhoto = false;
    appData.photoReady = false;
    appData.pollPhoto = false;
    appData.fileOpened = false;
           
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_INIT">
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            if(!appData.isTmr)
            {
                //Initialise the timer stuff    
                PLIB_TMR_ClockSourceSelect(TMR_ID_1, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
                PLIB_TMR_PrescaleSelect(TMR_ID_1, TMR_PRESCALE_VALUE_256);
                PLIB_TMR_Counter16BitClear(TMR_ID_1);
                PLIB_TMR_Period16BitSet(TMR_ID_1, 0xFFFF);
                PLIB_TMR_Start(TMR_ID_1);
                appData.isTmr = true;
            }
            
            // <editor-fold defaultstate="collapsed" desc="Open the various I2C and SPI drivers">
            
            if(appData.arducamI2CHandle == DRV_HANDLE_INVALID)
            {
                appData.arducamI2CHandle = DRV_I2C_Open( DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);
            }
            
            // If all drivers are valid, do any specific config and move to next state machine
            if(appData.arducamI2CHandle != DRV_HANDLE_INVALID)  
            {
                appData.state = APP_STATE_SETUP_ARDUCAM;
            }
            
            break;
        } //case APP_STATE_INIT
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_TEST_SPI">
        case APP_STATE_SETUP_ARDUCAM:
        {
            
            if(!appData.spiDone)
            {
                
                arducam_spi_init();
                
                // we're going to send 0x55 to test register 0x00
                uint8_t   Data[] = { 0x00, 0x55 };
                uint8_t   testReg1, testReg2;

                // send test value 1 and read it back (should return 0x77)
                arducam_write_reg(Data[0], Data[1]);
                testReg1 = arducam_read_reg(Data[0]);

                // Change MCU mode
                arducam_write_reg(0x02, 0x00); //MCU MODE (0x01 is Capture mode)
                 
                //if successful light led 
                if(testReg1 == Data[1])
                {
                    BSP_LED_8Toggle();
                    char SPIresult[40];
                    sprintf(SPIresult, "\n{ written: %d, received: %d, version no: %d }\n", Data[1], testReg1, modeReg);
                    msgStatus(SPIresult);
                    
                    char *SPISuccess = "Successfully written/read arducam SPI test register\n\n";
                    msgStatus(SPISuccess);
                }

                appData.spiDone = true;
            } 

            if(!appData.isI2CSent) 
            {  
                // Check if the camera module type is OV2640 (vid=0x26, pid=0x42) 
                arducam_i2c_read(OV2640_CHIPID_HIGH, &appData.vid);
                arducam_i2c_read(OV2640_CHIPID_LOW, &appData.pid); 
                appData.isI2CSent = true;
            }
            
            if(appData.isI2CReceived)
            { 
                if(appData.pid != 0x00)
                {
                    char I2Cresult[40];
                    sprintf(I2Cresult, "\n{ Vid: %d, Pid: %d }\n", appData.vid, appData.pid);
                    msgStatus(I2Cresult);
                    
                    appData.isI2CReceived = false;
                    if((appData.vid == 0x26) && (appData.pid == 0x42))
                    {
                        char *I2CSuccess = "OV2640 Camera module successfully detected!\n";
                        msgStatus(I2CSuccess);
                    
                        arducam(smOV2640);
                        arducam_set_format(fmtJPEG);
                        arducam_init();
                        
                        // Give arducam 1s to autofocus, etc after set up.
                        int i = 0;
                        while(i<10)
                        {
                            Delay_ms(100);
                            i++;
                        }
                        
                        char *I2CInitialised = "OV2640 Camera module successfully initialised!\n\n";
                        msgStatus(I2CInitialised);
                    
                        
                    } else {
                        char *I2CFailure = "Camera module reports incorrect vid/pid!\n\n";
                        msgStatus(I2CFailure);
                    }
                }
            }
            appData.state = APP_STATE_MOUNT_DISK;

            break;
        }  // </editor-fold>        
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_MOUNT_DISK">
        case APP_STATE_MOUNT_DISK:
        {
            
            if(appData.isMounted)
            {
                // already successfully mount previously, skip drive setup stuff and go 
                // to "normal" state.
                appData.state = APP_STATE_NORMAL;
            } else {
                
            
                if(SYS_FS_Mount("/dev/mmcblka1", "/mnt/myDrive", FAT, 0, NULL) != 0)
                {
                    // The disk could not be mounted. Try mounting again until success. 
                    
                        if(appData.j > 10000)
                        {
                            char *MNT_FAILED = "After 10000 state machine cycles, failed to mount /dev/mmcblka1\n";
                            msgStatus(MNT_FAILED);
                            
                            
                            
                            appData.j = 0;
                            
                            appData.state = APP_STATE_MOUNT_DISK;

                        } else {  
                            appData.j++;
                            appData.state = APP_STATE_MOUNT_DISK;
                        }
                    
                } else { 
                    // Mount was successful. Send a message and proceed to drive stuff...
                    char *FS_MNT_SUCCESSFUL = "/dev/mmcblka1 successfully mounted to /mnt/myDrive\n";
                    msgStatus(FS_MNT_SUCCESSFUL);
                    appData.j = 0;
                    mountSuccessLCD();
                    appData.isMounted = true;
                    
                    appData.state = APP_STATE_SET_CURRENT_DRIVE;
                    
                }
            }
            
            break;
        } //case APP_STATE_MOUNT_DISK
        // </editor-fold>     
      
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_SET_CURRENT_DRIVE">
        case APP_STATE_SET_CURRENT_DRIVE:
        {
            if(SYS_FS_CurrentDriveSet("/mnt/myDrive") == SYS_FS_RES_FAILURE)
            {
                /* Error while setting current drive */
                char *FS_SET_CURRENT_DRIVE_FAILED = "failed to set /mnt/myDrive as current drive\n";
                msgStatus(FS_SET_CURRENT_DRIVE_FAILED);
                appData.state = APP_STATE_UNMOUNT_DISK;
            }
            else
            {
                /* Open a file for reading. */
                char *FS_SET_CURRENT_DRIVE_SUCCESSFUL = "/mnt/myDrive successfully set as current drive\n";
                msgStatus(FS_SET_CURRENT_DRIVE_SUCCESSFUL);
                appData.state = APP_STATE_NORMAL;
            }
            break;
        } // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_UNMOUNT_DISK">
        case APP_STATE_UNMOUNT_DISK:
        {
            if(SYS_FS_Unmount("/mnt/myDrive") != 0)
            {
                /* The disk could not be un mounted. Try
                 * un mounting again until success.*/ 
        
                appData.state = APP_STATE_UNMOUNT_DISK;
            }
            else
            {
                /* UnMount was successful. Mount the disk again */
                char *FS_UNMNT_SUCCESSFUL = "/mnt/myDrive successfully unmounted\n";
                msgStatus(FS_UNMNT_SUCCESSFUL);
                appData.state = APP_STATE_NORMAL;
            }
            break;
        } // case APP_STATE_UNMOUNT_DISK //</editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_NORMAL">
        case APP_STATE_NORMAL:
        {
            //Everything comes back to here...
            appData.state = APP_STATE_TAKE_PHOTO;            // change state to RX buffer analysis
            
            break;
        } //case APP_STATE_NORMAL//</editor-fold>
        
        case APP_STATE_TAKE_PHOTO:
        {
            if(appData.takePhoto)
            {
                 char *ARDUCAM_TAKE_PHOTO = "Starting photo capture\n";
                msgStatus(ARDUCAM_TAKE_PHOTO);
                arducam_flush_fifo();    
                 // Clear the capture done flag
                arducam_clear_fifo_flag();
                // Start capture
     
                arducam_start_capture();
                
                appData.takePhoto = false;
                appData.pollPhoto = true;
                       
                
            } 
            appData.state = APP_STATE_PHOTO_READY;
            break;
           
        }
                 
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_PHOTO_READY">
        case APP_STATE_PHOTO_READY:
        {
            if(appData.pollPhoto = true)
            {
                uint8_t test;
                test = arducam_spi_bus_read(0x41);
                if(test == 0x08)
                {
                    
                    appData.photoReady = true;
                    break;
                }
            }
            
            if(appData.photoReady)
            {
                
                appData.pollPhoto = false;
                appData.state = APP_STATE_OPEN_FILE;
                
                break;
            } else {
                appData.state  = APP_STATE_ANALYSE_RX_BUFFER;
                break;
            }
        }
        
         // <editor-fold defaultstate="collapsed" desc="APP_STATE_OPEN_FILE">
        case APP_STATE_OPEN_FILE:
        {
            if(!appData.fileOpened)
            {
                appData.jpgHandle1 = SYS_FS_FileOpen("arduTest2.jpg",
                        (SYS_FS_FILE_OPEN_WRITE));
                if(appData.jpgHandle1 == SYS_FS_HANDLE_INVALID)
                {
                    char *FS_READ_FAILURE = "failed to open handle to arduTest.jpg\n";
                    msgStatus(FS_READ_FAILURE);
                    /* Could not open the file. Error out*/
                    appData.state = APP_STATE_ANALYSE_RX_BUFFER;
                }
                else
                {
                    char *FS_READ_SUCCESS = "successfully opened a handle to arduTest.jpg\n";
                    msgStatus(FS_READ_SUCCESS);
                    /* Create a directory. */
                    appData.fileOpened = true;
                    appData.state = APP_STATE_SAVE_PHOTO;
                }
            }
            break;
        } // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_SAVE_PHOTO">
        case APP_STATE_SAVE_PHOTO:
        {
            
            char *ARDUCAM_PHOTO_READY = "Arducam capture complete - writing to file\n";
            msgStatus(ARDUCAM_PHOTO_READY);
            //find image size in bytes
            size_t photoLength;
            photoLength = read_fifo_length();
            
            char FileLength[40];
            sprintf(FileLength, "\n{ image length in bytes: %d }\n", photoLength);
            msgStatus(FileLength);
            
            while(photoLength > 0)
            {
                if(photoLength >= 256)
                {
                    arducam_spi_burst_read(appData.fileData, 256);
                    
                    // If read was success, try writing to the new file 
                    if(SYS_FS_FileWrite(appData.jpgHandle1, (const void *)appData.fileData, 256) == -1)
                    {
                    // Write was not successful. Close the file
                    // and error out.
                        char *FS_WRITE_FAILURE = "arduTest.jpg write failed\n";
                        msgStatus(FS_WRITE_FAILURE);
                        break;
                    } else {
                        photoLength = photoLength - 256;
                    }
                } else {
                    arducam_spi_burst_read(appData.fileData, photoLength);
                    
                    int j = 0;
                    bool done = false;
                    
                    while (j<photoLength-1 || done != true)
                    {
                        if(     (j > 0) && 
                                (appData.fileData[j] == 0xD8) && 
                                (appData.fileData[j-1] == 0xFF)); // if last two bytes were
                        {                                         // 0xFF and 0xD8, we're  
                            photoLength = j + 1;                  // at the end of the jpg data
                            done = true;                          // so set the final data segment to
                        }                                         // this value...  
                        j++;
                    }
                    
                    if(SYS_FS_FileWrite(appData.jpgHandle1, (const void *)appData.fileData, photoLength) == -1)
                    {
                    // Write was not successful. Close the file
                    // and error out.
                        char *FS_WRITE_FAILURE = "arduTest.jpg write failed\n";
                        msgStatus(FS_WRITE_FAILURE);
                        break;
                    } else {
                        photoLength = 0;
                        char *ARDUCAM_PHOTO_COMPLETE = "finished writing arduTest.jpg\n\n";
                        msgStatus(ARDUCAM_PHOTO_COMPLETE);
                        arducam_flush_fifo();    
                        // Clear the capture done flag
                        arducam_clear_fifo_flag();
                        appData.photoReady = false;
                    }

                   
                }
            }
        
            SYS_FS_FileClose(appData.jpgHandle1);
            appData.state = APP_STATE_ANALYSE_RX_BUFFER;
            break;

        } //</editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_ANALYSE_RX_BUFFER">
        case APP_STATE_ANALYSE_RX_BUFFER:
        {      
            if(!isEmpty(&appData.rxRingBuffer))
            {
               
                while(!isEmpty(&appData.rxRingBuffer))
                {
                    char rxDat;
                    getItem(&appData.rxRingBuffer, &rxDat);
                    putItem(&appData.txRingBuffer, rxDat);
                    putItem(&appData.LCDRingBuffer, rxDat);
                    appData.logData[appData.nCharslogFile] = rxDat;

                }  

            }
            
            appData.state = APP_STATE_TX_BUFFER_SEND;
            break;
            
        } //case APP_STATE_ANALYSE_RX_BUFFER </editor-fold>
               
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_TX_BUFFER_SEND">
        case APP_STATE_TX_BUFFER_SEND:
        {
            
           // check if transmit buffer empty, if not, send bytes:
           while(!isEmpty(&appData.txRingBuffer)) 
           {
               if(!DRV_USART_TransmitBufferIsFull(appData.usartHandle))
               {
                   char dat;
                   getItem(&appData.txRingBuffer, &dat);
                   DRV_USART_WriteByte(appData.usartHandle, dat);
               }
           }  

            // Jump to LCD buffer handler.
           appData.state = APP_STATE_WRITE_LCD_MSG;          
           break;
        } //case APP_STATE_TX_BUFFER_SEND </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="APP_STATE_WRITE_LCD_MSG">
        case APP_STATE_WRITE_LCD_MSG:
        {
            unsigned char LCDdat;
            while(!isEmpty(&appData.LCDRingBuffer))
            {
                
                getItem(&appData.LCDRingBuffer, &LCDdat);
                if(LCDdat >= 0x20)
                {
                    writeToLCD(DATAREG, LCDdat);
                }
            }
            
            // Jump back to normal state machine.
            appData.state = APP_STATE_NORMAL;
            break;
        } //case APP_STATE_WRITE_LCD_MSG </editor-fold>
       
        // <editor-fold defaultstate="collapsed" desc="APP_STATE_ERROR">
        case APP_STATE_ERROR:
        {
            Delay_ms(250);
            BSP_LED_8Toggle();
            appData.state = APP_STATE_ERROR;
            break;
        } //</editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="default">

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        } // </editor-fold>
    }
}

void msgStatus(char *(msg))
{
    while(*msg)
    {
        putItem(&appData.txRingBuffer, *msg++);
    }
    return;
}

/*******************************************************************************
 End of File
 */
