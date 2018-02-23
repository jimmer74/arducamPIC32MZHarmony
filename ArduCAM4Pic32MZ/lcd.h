/* 
 * File:   lcd.h
 * Author: jim
 *
 * 
 * TO DO: Implement 
 * 1) More functions: shift right, shift left, move cursor
 * 2) remove delays after Initialisation. Use PMP busy to send if ready.
 * 
 * 
 * Created on 09 January 2018, 21:16
 */

#ifndef LCD_H
#define	LCD_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "app.h"
#include "delay.h"
APP_DATA appData;
    
/* Define aliases for the LCD data and command registers */
#define DATAREG 1 /* Data register */
#define CMDREG 0  /* Command register */
    
        
/*
 LCD function prototypes
 */    


void initializeLCD(void);
void writeToLCD(int reg, char c);
void writeString(unsigned char *string);
int readFromLCD(int reg);

void firstlineLCD(void);
void secondlineLCD(void);
void clearLCD(void);   
void moveCursorLCD(char direction);
void shiftDisplayLCD(char direction);

void greetingLCD(void);
void mountSuccessLCD(void);
void unmountSuccessLCD(void);
void mountErrorLCD(void);
void unmountErrorLCD(void);
void fileCopiedLCD(void);
void camSpiErrorLCD(char *temp);


#ifdef	__cplusplus
}
#endif

#endif	/* LCD_H */

