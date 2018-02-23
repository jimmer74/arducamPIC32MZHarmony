/* 
 * File:   delay.h
 * Author: jim
 *
 * Created on 06 February 2018, 16:24
 */

#ifndef DELAY_H
#define	DELAY_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "app.h"
  
#define CPU_CLOCK_HZ             (200000000UL)    // CPU Clock Speed in Hz 
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz 
#define PERIPHERAL_CLOCK_HZ      (100000000UL)    // Peripheral Bus  in Hz 

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks 

    
    
void Delay_ms(unsigned int x);
void Delay_40us();
void Delay_s(unsigned int x);
void Delay_nop(unsigned int x);

#ifdef	__cplusplus
}
#endif

#endif	/* DELAY_H */

