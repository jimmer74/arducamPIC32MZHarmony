#include "delay.h"


/*******************************************************************************

  Function:
    void Delay_ms (unsigned int x)

  Summary:
    Delay needed for the LCD to process data (in ms)
*/
void Delay_ms(unsigned int x)
{
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < (252 * x)); /* 252 Timer 1 ticks = ~1ms */
}

void Delay_40us()
{
    PLIB_TMR_Counter16BitClear(TMR_ID_1); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_1) < 40);
}

void Delay_nop(unsigned int x)
{
    int i = 0;
    while (i<x)
    {
        asm("nop");
        i++;
    }
    
}

void Delay_s(unsigned int x)
{
    int i, j = 0;
    while(i < x)
    {
        while(j<1000)
        {
            Delay_ms(1);
            j++;
        }
        
        i++;
    }
}
