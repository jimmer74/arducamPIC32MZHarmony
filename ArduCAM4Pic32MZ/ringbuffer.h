/* 
 * File:   ringbuffer.h
 * Author: jim
 *
 * Created on 10 January 2018, 09:18
 */

#ifndef RINGBUFFER_H
#define	RINGBUFFER_H

#ifdef	__cplusplus
extern "C" {
#endif
 
// Note power of two buffer size
#define kNumPointsInMyBuffer 512 

typedef struct ringBuffer_s
{
    int     first;
    int     last;
    int     validItems;
    char    data[kNumPointsInMyBuffer];
} ringBuffer_t;

void initialiseRingBuffer(ringBuffer_t *ringBuffer);
int isEmpty(ringBuffer_t *ringBuffer);
int putItem(ringBuffer_t *ringBuffer, char data);
int getItem(ringBuffer_t *ringBuffer, char *data);
void printRingBuffer(ringBuffer_t *ringBuffer);
uint32_t* returnRingBuffer(ringBuffer_t *ringBuffer, uint32_t Buff[]);
int validItemsRingBuffer(ringBuffer_t *ringBuffer, int i);
#ifdef	__cplusplus
}
#endif

#endif	/* RINGBUFFER_H */

