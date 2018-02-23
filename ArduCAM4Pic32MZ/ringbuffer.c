
/* 
 * File:   ringbuffer.c
 * Author: jim
 *
 * 
 * TO DO: Implement 
 * 1) Properly comment & Tidy up code - more human readable is better!
 * 2) Remove modulus and replace with (ringBuffer->last+1) & (kNumPointsInMyBuffer - 1) 
 *      - or something similar.    DONE!
 * 
 * Created on 09 January 2018, 21:16
 */



/*******************************************************************************
 * Functions to handle usart ringbuffer
*/

//Standard C libraries
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

//Microchip PIC32 Libraries
#include "system_config.h"
#include "system_definitions.h"

//User Created Libraries
#include "ringbuffer.h"

void initialiseRingBuffer(ringBuffer_t *ringBuffer)
{
    int i;
    
    /******************************************************************
     * create a ring buffer with the following initial conditions:    *
     * 1) No validItems - so empty                                    *
     * 2) first/read and last/write pointers are set at zero          *
     * 3) every position from zero to buffer length is zeroed         *
     ******************************************************************/
    ringBuffer->validItems = 0;
    ringBuffer->first = 0;
    ringBuffer->last = 0;
    for(i=0; i<kNumPointsInMyBuffer; i++)
    {
        ringBuffer->data[i] = 0;
    }
    return;
}

int validItemsRingBuffer(ringBuffer_t *ringBuffer, int i)
{
    i = ringBuffer->validItems;
    return(i);
}

int isEmpty(ringBuffer_t *ringBuffer)
{
    if(ringBuffer->validItems==0) // validItems/Number of items in buffer is zero, so return 1/true
        return(1);
    else
        return(0); // there are items so return false/0
}

int putItem(ringBuffer_t *ringBuffer, char theItemValue)
{
    if(ringBuffer->validItems>=kNumPointsInMyBuffer) // do we need this with our model? Perhaps when we are
                                                     // talking about image data, then we might need it later
                                                     // - or need a rewrite! I'll leave it here till then.
    {
        //something has gone wrong if we are here
        return(-1);
    }
    else
    {
        ringBuffer->validItems++; // we're adding an item to buffer, increase buffer full size/validItems to reflect this
        ringBuffer->data[ringBuffer->last] = theItemValue; // add our item/theItemValue at the last/write pointer position
        ringBuffer->last = (ringBuffer->last+1) & (kNumPointsInMyBuffer - 1); // moves the last/write pointer one place
                                                                              // on and wraps around if pointer
                                                                              // value greater than kNumPointsInMyBuffer
    }
}

int getItem(ringBuffer_t *ringBuffer, char *theItemValue)
{
    if(isEmpty(ringBuffer))
    {
        return(-1);
    }
    else
    {
        *theItemValue=ringBuffer->data[ringBuffer->first]; //grabs the first item
        ringBuffer->first=(ringBuffer->first+1) & (kNumPointsInMyBuffer - 1); // moves the first/read pointer one place
                                                                              // on and wraps around if pointer
                                                                              // value greater than kNumPointsInMyBuffer
        ringBuffer->validItems--; // we've popped a byte off the buffer and moved the pointer on, so one less valid item available
        return(0);
    }
}

void printBuffer(ringBuffer_t *ringBuffer)
{
    int aux, aux1;
    aux  = ringBuffer->first;
    aux1 = ringBuffer->validItems;
    while(aux1>0)
    {
        printf("Element #%d = %d\n", aux, ringBuffer->data[aux]);
        aux=(aux+1) & (kNumPointsInMyBuffer - 1);
        aux1--;
    }
    return;
}

uint32_t* returnRingBuffer(ringBuffer_t *ringBuffer, uint32_t Buff[512] )
{
    
    int aux, aux1, i;
    aux  = ringBuffer->first;
    aux1 = ringBuffer->validItems;
    i = 0;
    while(aux1>0)
    {
       
        Buff[i] = ringBuffer->data[aux];
        aux=(aux+1) & (kNumPointsInMyBuffer - 1);
        aux1--;
        i++;
        if(aux1 == 1)
        {
            Buff[i+1] = '\0';
        }
    }
    
    ringBuffer->first = aux;
    ringBuffer->last = aux;
    ringBuffer->validItems = 0;
    return Buff;
}