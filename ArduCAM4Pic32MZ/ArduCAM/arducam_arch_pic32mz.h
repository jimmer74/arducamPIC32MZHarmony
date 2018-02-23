/*-----------------------------------------
 * Update History:
 * 2016/06/13 	V1.1	by Lee	
 *      add support for burst mode
 * 
 * Modified on: 2018.02.06
 *      Author: Jim Wiley
 *      Porting to PIC32MZ Harmony implementation
--------------------------------------*/

#ifndef __ARDUCAM_ARCH_PIC32MZ_H__
#define __ARDUCAM_ARCH_PIC32MZ_H__

#include "../app.h"

void arducam_spi_init(void);
void arducam_i2c_init(void);


void arducam_spi__bus_write(uint8_t address, uint8_t value);
uint8_t arducam_spi_bus_read(uint8_t address);

void arducam_spi_transfer(uint8_t data);
void arducam_spi_transfers(uint8_t *txBuf, uint32_t txSize, uint8_t *rxBuf);
void arducam_spi_burst_read(uint8_t *rxBuf, size_t len);


// Read/write 8 bit value to/from 8 bit register address
uint8_t arducam_i2c_write(uint8_t regID, uint8_t regDat);
uint8_t arducam_i2c_read(uint8_t regID, uint8_t* regDat);

// Read/write 16 bit value to/from 8 bit register address
uint8_t arducam_i2c_write16(uint8_t regID, uint16_t regDat);
uint8_t arducam_i2c_read16(uint8_t regID, uint16_t* regDat);

// Read/write 8 bit value to/from 16 bit register address
uint8_t arducam_i2c_word_write(uint16_t regID, uint8_t regDat);
uint8_t arducam_i2c_word_read(uint16_t regID, uint8_t* regDat);

// Write 8 bit values to 8 bit register address
int arducam_i2c_write_regs(const struct sensor_reg reglist[]);

// Write 16 bit values to 8 bit register address
int arducam_i2c_write_regs16(const struct sensor_reg reglist[]);

// Write 8 bit values to 16 bit register address
int arducam_i2c_write_word_regs(const struct sensor_reg reglist[]);

void I2CMasterReadRegCb(DRV_I2C_BUFFER_EVENT event,
                            DRV_I2C_BUFFER_HANDLE bufferHandle,
                            uintptr_t context);
void I2CMasterWriteRegCb(DRV_I2C_BUFFER_EVENT event,
                            DRV_I2C_BUFFER_HANDLE bufferHandle,
                            uintptr_t context);

#endif 