/* 
 * File:   lcd.c
 * Author: jim
 *
 * 
 * TO DO: Implement 
 * 1) More functions: shift right, shift left, move cursor
 * 2) remove delays after Initialisation. Use PMP busy to send if ready. 
 *        - worked out 20 ticks is approx 80us, so used this instead. DONE
 * 
 * 
 * Created on 09 January 2018, 21:16
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "system_config.h"
#include "system_definitions.h"

#include "app.h"
#include "lcd.h"

 
extern APP_DATA appData;




/*******************************************************************************

  Function:
    void initializeLCD (void)

  Summary:
    LCD initialization sequence
*/
void initializeLCD(void)
{
    /* Configure the PMP options - possibly already done by harmony elsewhere? */
    PLIB_PMP_AddressSet(PMP_ID_0, PMP_PMA0_PORT);
    PLIB_PMP_AddressPortEnable(PMP_ID_0, PMP_PMA0_PORT);

    Delay_ms(70); /* LCD needs 60ms to power on and perform startup functions 
                  but LCD is already powered while the start up stuff is performed
                  so I experimented and found I only needed 1ms if I initialise the
                  LCD at this point in the code. Should probably be changed back
                  for another project */

    PLIB_PMP_AddressSet(PMP_ID_0, CMDREG); /* Access the LCD command register */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x38); /* LCD Function Set - 8-bit interface, 2 lines, 5*7 Pixels */
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (20)); /* Needs a 40us delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x0C); /* Turn on display (with cursor hidden) */
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (20)); /* Needs a 40us delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x01); /* Clear the display */
    Delay_ms(2); /* Needs a 1.64ms delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x06); /* Set text entry mode - auto increment, no shift */
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (225)); /*Needs this many ticks, 
                                                         * any less and it's not ready
                                                         approx 0.4ms?*/
    appData.lcdInitialised = true;
    

}



/*******************************************************************************

  Function:
    void writeToLCD(int reg, char c)

  Summary:
    Writes a byte of data to either of the two LCD registers (DATAREG, CMDREG)
*/
void writeToLCD(int reg, char c)
{
    
    if(appData.xcursorLCD > 16 && appData.ycursorLCD == 0)
    {         
        secondlineLCD();
    } 
    else if(appData.xcursorLCD > 16 && appData.ycursorLCD == 1)
    {  
        firstlineLCD();    
    }
       
    appData.xcursorLCD++;
    
    PLIB_PMP_AddressSet(PMP_ID_0, reg); /* Either 'DATAREG' or 'CMDREG' */
    
    DRV_PMP0_Write(c);
    
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (20)); // 10 ticks = 40 us?
           
}
    

int readFromLCD(int reg)
{
//  
    PLIB_PMP_AddressSet(PMP_ID_0, reg);
    
    char dat = PLIB_PMP_MasterReceive(PMP_ID_0);
    Delay_40us();
    return dat;
    //while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (40)); // 10 ticks = 40 us?*/
            //DRV_PMP_Master 0_Read(); //return second PMP read.
}
/*******************************************************************************

  Function:
    void writeString (unsigned char *string)

  Summary:
    Used to write text strings to the LCD
*/
void writeString(unsigned char *string)
{
    while (*string)
    {
        writeToLCD(DATAREG, *string++); /* Send characters one by one */
    }
}


/*******************************************************************************

  Function:
    void secondlineLCD(void)

  Summary:
    Sets the LCD cursor position to line two

*/
void secondlineLCD(void)
{
    appData.ycursorLCD = 1;
    appData.xcursorLCD = 0;
    writeToLCD(CMDREG, 0xC0); /* Cursor address 0x80 + 0x40 = 0xC0 */
    Delay_ms(2);
}
/*
 Function:
    void firstlineLCD(void)

  Summary:
    Sets the LCD cursor position to line one

*/
void firstlineLCD(void)
{
    appData.ycursorLCD = 0;
    appData.xcursorLCD = 0;
    writeToLCD(CMDREG, 0x02); // line 1, column 1. NOTE: needs 1.56ms to complete
    Delay_ms(2);
}
 
/*
 Function:
    void clearLCD(void)

  Summary:
    Sets the LCD cursor position to line one

*/

void clearLCD(void)
{
    appData.ycursorLCD = 0;
    appData.xcursorLCD = 0;
    writeToLCD(CMDREG, 1); // clears the LCD Screen. NOTE: needs 1.56ms to complete
     Delay_ms(2);
}

void greetingLCD(void)
{
    //clearLCD();
    writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"Please Insert");
    secondlineLCD();
    writeString((unsigned char *)"SDCard...");
}

void mountErrorLCD(void)
{
    clearLCD();
    //writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"Mount");
    secondlineLCD();
    writeString((unsigned char *)"Error");
}

void unmountErrorLCD(void)
{
    clearLCD();
    //writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"Unmounting");
    secondlineLCD();
    writeString((unsigned char *)"Error");
}

void fileCopiedLCD(void)
{
    firstlineLCD();
    //writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"File Copied     ");
    secondlineLCD();
    writeString((unsigned char *)"Successfully... ");
}

void mountSuccessLCD(void)
{
    clearLCD();
    writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"SDCard");
    secondlineLCD();
    writeString((unsigned char *)"Mounted");
}

void unmountSuccessLCD(void)
{
    clearLCD();
    //writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"SDCard unmounted");
    secondlineLCD();
    writeString((unsigned char *)"Successfully");
}

void camSpiErrorLCD(char *temp)
{
    clearLCD();
    //writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
    writeString((unsigned char *)"Temp value:");
    secondlineLCD();
    writeString(temp);
}

void moveCursorLCD(char direction)
{
    switch( direction )
    {
        case 'R':
        {
            writeToLCD(CMDREG, 0x14);
            break;
        }
        
        case 'L':
        {
            writeToLCD(CMDREG, 0x14);
            break;
        }
    }
}

void shiftDisplayLCD(char direction)
{
    switch (direction)
    {
        case 'R':
        {
            writeToLCD(CMDREG, 0x1C);
            break;
        }
        
        case 'L':
        {
            writeToLCD(CMDREG, 0x18);
            break;
        }
    }
}