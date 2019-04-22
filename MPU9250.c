/*
 * MPU9250.c
 *
 *  Created on: 12 de abr de 2019
 *      Author: Guilherme Silva Lionço
 */

#include "MPU9250.h"


#define EUSCI_I2C_MODULE                  EUSCI_B1_BASE
#define EUSCI_I2C_PORT                    GPIO_PORT_P6
#define EUSCI_I2C_SCL_PIN                 GPIO_PIN5
#define EUSCI_I2C_SCL_PIN_FUNCTION        GPIO_PRIMARY_MODULE_FUNCTION
#define EUSCI_I2C_SDA_PIN                 GPIO_PIN4
#define EUSCI_I2C_SDA_PIN_FUNCTION        GPIO_PRIMARY_MODULE_FUNCTION
#define EUSCI_I2C_STATUS_SLAVE_NACK       1

void initI2C(void);
void setSleepEnabled(bool enabled);
void setFullScaleGyroRange(uint8_t range);
void setFullScaleAccelRange(uint8_t range);
void MPU9250_setClockSource(uint8_t source);
void writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
void writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount);
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount);
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU9250_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);

uint8_t MPU9250_devAddr = 0x68;

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

void MPU9250_initialize(){

    initI2C();

    MPU9250_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false);

}
//check whether MPU9250//6050 is available on the I2C BUS
bool checkConnection(){

    uint8_t data;

    readI2C(MPU9250_devAddr,0x75, &data, 1);

    if(data==MPU9250_devAddr||data==0x73){
        return true;
    }else{
        return false;
    }

}

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU9250_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz) {
    getMotion6(ax, ay, az, gx, gy, gz);
    // TODO: magnetometer integration
}

void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    readI2C(MPU9250_devAddr,MPU6050_RA_ACCEL_XOUT_H,&buffer,14);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void setSleepEnabled(bool enabled) {
    writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void setFullScaleAccelRange(uint8_t range) {
    writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void setFullScaleGyroRange(uint8_t range) {
    writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
*/
void MPU9250_setClockSource(uint8_t source) {

    writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

void writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {

    uint8_t b;

    readI2C(MPU9250_devAddr,regAddr, &b, 1);

    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    writeI2C(MPU9250_devAddr, regAddr, &b, 1);


}
void writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){

    uint8_t b=0;

    readI2C(MPU9250_devAddr,regAddr, &b, 1);

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);

    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    writeI2C(MPU9250_devAddr, regAddr, &b, 1);

}

void initI2C(void)
{
    /* I2C Clock Soruce Speed */
    //i2cConfig.i2cClk = CS_getSMCLK();

    /* Select I2C function for I2C_SCL & I2C_SDA */
    GPIO_setAsPeripheralModuleFunctionOutputPin(EUSCI_I2C_PORT, EUSCI_I2C_SCL_PIN,
            EUSCI_I2C_SCL_PIN_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(EUSCI_I2C_PORT, EUSCI_I2C_SDA_PIN,
            EUSCI_I2C_SDA_PIN_FUNCTION);

    /* Initializing I2C Master to SMCLK at 400kbs with no-autostop */
    MAP_I2C_initMaster(EUSCI_I2C_MODULE, (const eUSCI_I2C_MasterConfig *)&i2cConfig);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_I2C_MODULE);
}

/***********************************************************
  Function:
*/
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount)
{
    /* Todo: Implement a delay */
    /* Wait until ready to write */
    while (MAP_I2C_isBusBusy(EUSCI_I2C_MODULE));

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_I2C_MODULE, ui8Addr);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_I2C_MODULE,ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Now write one or more data bytes */
    while(1)
    {
        /* Wait for next INT */
        while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

        /* If no data to follow, we are done */
        if(ui8ByteCount == 0 )
        {
            MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
            MAP_I2C_clearInterruptFlag(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
            return(true);
        }
        /* If more, send the next byte */
        else
        {
            MAP_I2C_masterSendMultiByteNext(EUSCI_I2C_MODULE, *Data++);
        }
        ui8ByteCount--;
    }
}

/***********************************************************
  Function:
*/
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount)
{
    /* Todo: Implement a delay */
    /* Wait until ready */
    while (MAP_I2C_isBusBusy(EUSCI_I2C_MODULE));

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_I2C_MODULE, ui8Addr);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_I2C_MODULE,ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Turn off TX and generate RE-Start */
    MAP_I2C_masterReceiveStart(EUSCI_I2C_MODULE);

    /* Wait for start bit to complete */
    while(MAP_I2C_masterIsStartSent(EUSCI_I2C_MODULE));

    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Read one or more bytes */
    while(ui8ByteCount)
    {
        /* If reading 1 byte (or last byte), generate the stop to meet the spec */
        if(ui8ByteCount-- == 1)
        {
            *Data++ = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_I2C_MODULE);
        }
        else
        {
            /* Wait for next RX interrupt */
            while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_RECEIVE_INTERRUPT0) &
                    EUSCI_B_I2C_RECEIVE_INTERRUPT0));

            /* Read the rx byte */
            *Data++ = MAP_I2C_masterReceiveMultiByteNext(EUSCI_I2C_MODULE);
        }
    }

    MAP_I2C_clearInterruptFlag(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
    return(true);
}

/***********************************************************
  Function: euscib1IntHandler
 */
void euscib1IntHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_I2C_MODULE);
    MAP_I2C_clearInterruptFlag(EUSCI_I2C_MODULE, status);

    if (status & EUSCI_B_I2C_NAK_INTERRUPT)
    {
    }

    if (status & EUSCI_B_I2C_START_INTERRUPT)
    {
    }

    if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
    }

    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
    }

    if (status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0)
    {
    }
}

