/*-----------------------------------------

//Update History:
//2016/06/13 	V1.1	by Lee	add support for burst mode

--------------------------------------*/

#include "../app.h"
#include <string.h>

/*
 * These are included from "app.h" already...
 *
 *#include <stdint.h>
 *#include <stdlib.h>
 *#include <unistd.h>
 */

APP_DATA appData;
#include "arducam.h"
#include "arducam_arch_pic32mz.h"

//SPI Configuration
#define ARDUCAM_SPI_ID                      SPI_ID_6
#define ARDUCAM_BAUD_HZ                     1000000

#define ARDUCAM_SDI_PIN                     INPUT_PIN_RPE8
#define ARDUCAM_SDI_PORT                    PORT_CHANNEL_E
#define ARDUCAM_SDI_TRIS_BITS               TRISEbits.TRISE8 

#define ARDUCAM_SDO_PIN                     OUTPUT_PIN_RPF2
#define ARDUCAM_SDO_PORT                    PORT_CHANNEL_F
#define ARDUCAM_SDO_TRIS_BITS               TRISFbits.TRISF2

//I2C configuration 

#define ARDUCAM_I2C_ID                      I2C_ID_4
#define ARDUCAM_I2C_BAUD_HZ                 100000


#define ARDUCAM_I2C_PORT                    PORT_CHANNEL_G
#define ARDUCAM_SCL_TRIS_BITS               TRISGbits.TRISG8
#define ARDUCAM_SDA_TRIS_BITS               TRISGbits.TRISG7
    

#define ARDUCAM_I2C_8_BIT_ADDR         0x60 // 7-bit Addr shifted to 8-bit without R/W LSB Bit
#define ARDUCAM_I2C_7_BIT_ADDR         0x30
#define ARDUCAM_I2C_HAS_MASK           false
#define ARDUCAM_I2C_7_BIT_ADDR_MASK    0x30




// <editor-fold defaultstate="collapsed" desc="SPI FUNCTIONS">
/******************
 * 
 * SPI FUNCTIONS
 * 
 * 
 ******************/

// <editor-fold defaultstate="collapsed" desc="void arducam_spi_init(void)">
void arducam_spi_init(void)
{
    PLIB_SPI_Disable( ARDUCAM_SPI_ID );

    
    // This is fucked up, but apparently harmony is mishandling the input pin select,
    // so I've basically re-tooled it on the fly here so that I can actually receive 
    // data back when I read the camera register.

    //Set SDI6 Pins as Digital and add a pull up
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, ARDUCAM_SDI_PORT, 0x0000, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, ARDUCAM_SDO_PORT, 0x0000, PORTS_PIN_MODE_DIGITAL);
    
    ARDUCAM_SDI_TRIS_BITS = 1;   //SDI - Input
    ARDUCAM_SDO_TRIS_BITS = 0;   // SDO6 - Output
    
    //Map our pins to their SPI functions (define them above)
    PLIB_PORTS_RemapInput(PORTS_ID_0, INPUT_FUNC_SDI6, ARDUCAM_SDI_PIN );
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_SDO6, ARDUCAM_SDO_PIN );
    
    PLIB_SPI_MasterEnable ( ARDUCAM_SPI_ID  );
        
    PLIB_SPI_BaudRateClockSelect ( ARDUCAM_SPI_ID,  SPI_BAUD_RATE_PBCLK_CLOCK );
    PLIB_SPI_BaudRateSet( ARDUCAM_SPI_ID, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2), ARDUCAM_BAUD_HZ );
    
    
    PLIB_SPI_ClockPolaritySelect( ARDUCAM_SPI_ID, SPI_CLOCK_POLARITY_IDLE_LOW );
    PLIB_SPI_OutputDataPhaseSelect( ARDUCAM_SPI_ID, SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK );
    PLIB_SPI_InputSamplePhaseSelect ( ARDUCAM_SPI_ID,  SPI_INPUT_SAMPLING_PHASE_AT_END );
        
    
    PLIB_SPI_PinEnable( ARDUCAM_SPI_ID, SPI_PIN_DATA_OUT );
    PLIB_SPI_PinEnable( ARDUCAM_SPI_ID, SPI_PIN_DATA_IN );
    PLIB_SPI_PinEnable( ARDUCAM_SPI_ID, SPI_PIN_SLAVE_SELECT );    
      
    PLIB_SPI_FramedCommunicationDisable( ARDUCAM_SPI_ID );
    
    PLIB_SPI_StopInIdleEnable( ARDUCAM_SPI_ID );
    PLIB_SPI_FIFODisable( ARDUCAM_SPI_ID );
    
    PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
    PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    
    PLIB_SPI_Enable( ARDUCAM_SPI_ID );
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="void arducam_spi_bus_write(uint8_t address, uint8_t value)">
void arducam_spi_bus_write(uint8_t address, uint8_t value)
{
    PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
    PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    uint8_t rxdummybyte;
    
	SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
   
    PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, address);
    rxdummybyte = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);
    while(PLIB_SPI_IsBusy(ARDUCAM_SPI_ID))
    {
        ;
    }
    
    PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, value );
    rxdummybyte = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);              
    
    
    while(PLIB_SPI_IsBusy(ARDUCAM_SPI_ID))
    {
        ;
    }
    
    Delay_nop(5);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
    
    PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
    PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    
   
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_spi_bus_read(uint8_t address)">
uint8_t arducam_spi_bus_read(uint8_t address)
{
    //get a fresh buffer for this transaction
    PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
    PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    // Take CS line low
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
    Delay_nop(10);
    
    
    uint8_t txdummybyte, rxdummybyte, rb;
    txdummybyte = 0x00;
    rb = rxdummybyte = 0xFF;
    
    
    
    
    PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, address);
    while(!PLIB_SPI_ReceiverBufferIsFull(ARDUCAM_SPI_ID));
    if(PLIB_SPI_ReceiverBufferIsFull)
    {
        rxdummybyte = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);   
    }
       
    
    
    PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, txdummybyte );
    while(!PLIB_SPI_ReceiverBufferIsFull(ARDUCAM_SPI_ID));
    if(PLIB_SPI_ReceiverBufferIsFull)
    {
        rb = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);   
    }
    
    // Set the CS line high
    Delay_nop(30);
    SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
    
    
    PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
    PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    
    return rb;
 
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="void arducam_spi_transfers(uint8_t *txBuf, uint32_t txSize, uint8_t *rxBuf)">
void arducam_spi_transfers(uint8_t *txBuf, uint32_t txSize, uint8_t *rxBuf)
{
    
	 uint32_t rxSize = txSize;
     char txSwitch = 0;
     DRV_SPI_BUFFER_HANDLE spitxbufferHandle, spitxjobHandle;
    						
    switch ( txSwitch )
    {
        case 0:
        {
            spitxbufferHandle = DRV_SPI_BufferAddWriteRead2( appData.wiznetSPIHandle, txBuf, txSize,
                                                    rxBuf, rxSize, NULL, NULL, &spitxjobHandle);
            if(spitxjobHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
            {
                txSwitch++;
            }
            break;
        }
        
        case 1:
        {
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(spitxjobHandle))
            {
                txSwitch++;
            }
            break;
        }
        
        
        
    }

 
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="void arducam_spi_transfer(uint8_t txData)">
void arducam_spi_transfer(uint8_t txData)
{
    
    uint8_t txSize = sizeof(txData);
    char txSwitch = 0;
    DRV_SPI_BUFFER_HANDLE spitxbufferHandle, spitxjobHandle;
    						
    switch (txSwitch)
    {
        case 0:
        {
            spitxbufferHandle = DRV_SPI_BufferAddWrite2( appData.wiznetSPIHandle, &txData, txSize,
                                                            NULL, NULL, &spitxjobHandle);
            if(spitxjobHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
            {
                txSwitch++;
            }
            break;
        }
        
        case 1:
        {
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(spitxjobHandle))
            {
                txSwitch++;
            }
            break;
        }
       
    }
	
} //</editor-fold>

void arducam_spi_burst_read(uint8_t *rxBuf, size_t len)
{
   uint8_t txdummybyte = 0x00;
   uint8_t txcmdphase = 0x3c;
   uint8_t rxdummybyte;
   
   PLIB_SPI_BufferClear( ARDUCAM_SPI_ID );
   PLIB_SPI_ReceiverOverflowClear ( ARDUCAM_SPI_ID );
    // Take CS line low
   SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
   
   
   //Send 0x3c and then 
   PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, txcmdphase);
   while(!PLIB_SPI_ReceiverBufferIsFull(ARDUCAM_SPI_ID));
   if(PLIB_SPI_ReceiverBufferIsFull)
   {
       rxdummybyte = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);   // junk byte, discard.
   }
   
   int i;
   while(i<(len)) // send len dummy bytes and read len bytes in our buffer
   {
    PLIB_SPI_BufferWrite(ARDUCAM_SPI_ID, txdummybyte );
    while(!PLIB_SPI_ReceiverBufferIsFull(ARDUCAM_SPI_ID));
    if(PLIB_SPI_ReceiverBufferIsFull)
    {
        *rxBuf++ = PLIB_SPI_BufferRead(ARDUCAM_SPI_ID);
        i++;
    }
   }
    
   Delay_nop(10);
   
   SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_14);
}
//</editor-fold>


/******************
 * 
 * I2C FUNCTIONS
 * 
 * 
 ******************/

// <editor-fold defaultstate="collapsed" desc="void arducam_i2c_init(void)">
void arducam_i2c_init(void)
{
    //BSP_LED_5On();
    PLIB_I2C_Disable(ARDUCAM_I2C_ID);
   
        //Set SDA/SCL Pins as Digital
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, ARDUCAM_I2C_PORT, 0x0000, PORTS_PIN_MODE_DIGITAL); //Set all pins as digital
    
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8); // clear LAT bits
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);
 
    
    ARDUCAM_SCL_TRIS_BITS = 1;   //SCL - Output
    ARDUCAM_SDA_TRIS_BITS = 1;   // SDA - Input
    
    //I2C Features Enable/Disable    
    PLIB_I2C_HighFrequencyDisable(ARDUCAM_I2C_ID);
   
    PLIB_I2C_SMBDisable(ARDUCAM_I2C_ID);
    PLIB_I2C_StopInIdleDisable(ARDUCAM_I2C_ID);
    
    //I2C Baud rate set
    PLIB_I2C_BaudRateSet( ARDUCAM_I2C_ID, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2), ARDUCAM_I2C_BAUD_HZ);
    
    //Set Slave Address stuff
    PLIB_I2C_SlaveAddress7BitSet( ARDUCAM_I2C_ID, ARDUCAM_I2C_7_BIT_ADDR );
    if(ARDUCAM_I2C_HAS_MASK)
    {
        PLIB_I2C_SlaveMask7BitSet( ARDUCAM_I2C_ID, ARDUCAM_I2C_7_BIT_ADDR_MASK );
    }
    
    PLIB_I2C_Enable(ARDUCAM_I2C_ID);
    //BSP_LED_5Off();
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_write(uint8_t regID, uint8_t regDat)">
uint8_t arducam_i2c_write(uint8_t regID, uint8_t regDat)
{
    
    uint8_t Data[2];
    Data[0] = regID;
    Data[1] = regDat;
    
    DRV_I2C_BUFFER_HANDLE arducamI2CWriteBufferHandle;
    
    uintptr_t i2cWriteRegOpStatus; //Operation status of I2C operation returned from callback
    
    DRV_I2C_BufferEventHandlerSet(appData.arducamI2CHandle, 
                                        I2CMasterWriteRegCb, 
                                        i2cWriteRegOpStatus ); 
    BSP_LED_7Toggle();
    arducamI2CWriteBufferHandle = DRV_I2C_Transmit(appData.arducamI2CHandle,
                                               ARDUCAM_I2C_8_BIT_ADDR,
                                                Data,
                                                (sizeof(Data)),
                                                NULL);     
        
    while(DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(appData.arducamI2CHandle, 
                                                                    arducamI2CWriteBufferHandle))
    {
        asm("nop");
    }
    
    return 1;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_read(uint8_t regID, uint8_t* regDat)">
uint8_t arducam_i2c_read(uint8_t regID, uint8_t* regDat)
{
    
    DRV_I2C_BUFFER_HANDLE arducamI2CReadBufferHandle; 
    
    uintptr_t i2cReadRegOpStatus;  //Operation status of I2C operation returned from callback
    DRV_I2C_BufferEventHandlerSet(appData.arducamI2CHandle, 
                                        I2CMasterReadRegCb, 
                                        i2cReadRegOpStatus ); 
    
   
    arducamI2CReadBufferHandle = DRV_I2C_TransmitThenReceive(appData.arducamI2CHandle,
                                               ARDUCAM_I2C_8_BIT_ADDR,
                                                &regID,
                                                1,
                                                regDat,
                                                1,
                                                NULL);     
        
    while(DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(appData.arducamI2CHandle, 
                                                                    arducamI2CReadBufferHandle))
    {
        asm("nop");
    }

    
    
    return 1;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_write16(uint8_t regID, uint16_t regDat)">
uint8_t arducam_i2c_write16(uint8_t regID, uint16_t regDat)
{   
    /*
	if(FD != -1)
	{
		wiringPiI2CWriteReg16(FD,regID,regDat);
		
		return(1);
	}
     */
     
	return 0;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_read16(uint8_t regID, uint16_t* regDat)">
uint8_t arducam_i2c_read16(uint8_t regID, uint16_t* regDat)
{
    /*
	if(FD != -1)
	{
		*regDat = wiringPiI2CReadReg16(FD,regID);
		return(1);
	} */
	return 0;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_word_write(uint16_t regID, uint8_t regDat)">
uint8_t arducam_i2c_word_write(uint16_t regID, uint8_t regDat)
{   /*
	uint8_t reg_H,reg_L;
	uint16_t value;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	value =  regDat << 8 | reg_L;
	if(FD != -1)
	{
		i2c_smbus_write_word_data(FD, reg_H, value);
		//printf("regID:0x%x%x\t\tregDat:0x%02x\n",reg_H,reg_L,regDat);
		return(1);
	} */
	return 0;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="uint8_t arducam_i2c_word_read(uint16_t regID, uint8_t* regDat)">
uint8_t arducam_i2c_word_read(uint16_t regID, uint8_t* regDat)
{
    /*
	uint8_t reg_H,reg_L;
	int r;
	reg_H = (regID >> 8) & 0x00ff;
	reg_L = regID & 0x00ff;
	if(FD != -1)
	{
		r = i2c_smbus_write_byte_data(FD,reg_H,reg_L);
		if(r<0)
			return 0;
		*regDat = i2c_smbus_read_byte(FD);
		return(1);
	}
	return 0;
     */
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="int arducam_i2c_write_regs(const struct sensor_reg reglist[])">
int arducam_i2c_write_regs(const struct sensor_reg reglist[])
{
	uint16_t reg_addr = 0;
	uint16_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="int arducam_i2c_write_regs16(const struct sensor_reg reglist[])">
int arducam_i2c_write_regs16(const struct sensor_reg reglist[])
{
	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xff) | (reg_val != 0xffff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val = pgm_read_word(&next->val);
		if (!arducam_i2c_write16(reg_addr, reg_val)) {
			return 0;
		}
	   	next++;
	}

	return 1;
} //</editor-fold>

// <editor-fold defaultstate="collapsed" desc="int arducam_i2c_write_word_regs(const struct sensor_reg reglist[])">
int arducam_i2c_write_word_regs(const struct sensor_reg reglist[])
{
	unsigned int reg_addr,reg_val;
	const struct sensor_reg *next = reglist;

	while ((reg_addr != 0xffff) | (reg_val != 0xff))
	{
		reg_addr = pgm_read_word(&next->reg);
		reg_val =  pgm_read_word(&next->val);
		//if (!arducam_i2c_write16(reg_addr, reg_val)) 
		
		 //My changed
		if (!arducam_i2c_word_write(reg_addr, reg_val))
			{
			return 0;
            }
	   	next++;
	}

	return 1;
} //</editor-fold>


void I2CMasterReadRegCb (    DRV_I2C_BUFFER_EVENT event,
                            DRV_I2C_BUFFER_HANDLE bufferHandle,
                            uintptr_t context)
{
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            //this indicates that the I2C transaction has completed
            //DRV_I2C_BUFFER_EVENT_COMPLETE can be handled in the callback
            //or by checking for this event using the API DRV_I2C_BufferStatus
            /* include any callback event handling code here if needed */
            appData.isI2CReceived = true;
            
            
        break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            //this indicates that the I2C transaction has completed
            //and a STOP condition has been asserted on the bus.
            //However the slave has NACKED either the address or data
            //byte.
            /* include any callback event handling code here if needed */
            BSP_LED_9On();
        default:
        break;
    }

}

void I2CMasterWriteRegCb (    DRV_I2C_BUFFER_EVENT event,
                            DRV_I2C_BUFFER_HANDLE bufferHandle,
                            uintptr_t context)
{
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            //this indicates that the I2C transaction has completed
            //DRV_I2C_BUFFER_EVENT_COMPLETE can be handled in the callback
            //or by checking for this event using the API DRV_I2C_BufferStatus
            /* include any callback event handling code here if needed */
            BSP_LED_7Toggle();
            
            
        break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            //this indicates that the I2C transaction has completed
            //and a STOP condition has been asserted on the bus.
            //However the slave has NACKED either the address or data
            //byte.
            /* include any callback event handling code here if needed */
            BSP_LED_9On();
        default:
        break;
    }

}