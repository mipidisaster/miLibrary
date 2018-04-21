/**************************************************************************************************
 * @file        SPIDevice.cpp
 * @author      Thomas
 * @version     V0.2
 * @date        21 Apr 2018
 * @brief       Source file for the Generic SPIDevice Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "SPIDevice.h"

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
SPIDevice::SPIDevice(SPI_HandleTypeDef *SPIHandle) {
/**************************************************************************************************
 * Create a SPIDevice class specific for the STM32F device
 * Receives the address of the SPI Handle of device - generated from cubeMX
 * Will then determine which mode has been selected, based upon the states of the registers
 *************************************************************************************************/
    this->SPIHandle         = SPIHandle;    // copy handle across into class

    // From handle can determine what the MODE of the SPI can be configured too:
    if (SPIHandle->Instance->CR1 & SPI_POLARITY_HIGH) {     // If Clock Idles HIGH
        if (SPIHandle->Instance->CR1 & SPI_PHASE_2EDGE)     // If data is captured on second edge
            this->Mode = MODE3;                             // MODE is 3
        else                                                // Otherwise captured on first edge
            this->Mode = MODE2;                             // MODE is 2
    } else {                                                // If Clock Idles LOW
        if (SPIHandle->Instance->CR1 & SPI_PHASE_2EDGE)     // If data is captured on second edge
            this->Mode = MODE1;                             // MODE is 1
        else                                                // Otherwise captured on first edge
            this->Mode = MODE0;                             // MODE is 0
    }

    __HAL_SPI_ENABLE(SPIHandle);            // Enable the SPI interface
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
SPIDevice::SPIDevice(int channel, int speed, _SPIMode Mode) {
/**************************************************************************************************
 * Create a SPIDevice class specific for RaspberryPi
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    this->Mode          = Mode;         // Copy across the selected Mode
    this->SPIChannel    = channel;      //
    int tempMode = 0;

    if (this->Mode == MODE1)            // If Mode 1 is selected then
        tempMode = 1;                   // Store "1"
    else if (this->Mode == MODE2)       // If Mode 2 is selected then
        tempMode = 2;                   // Store "2"
    else if (this->Mode == MODE3)       // If Mode 3 is selected then
        tempMode = 3;                   // Store "3"
    else                                // If any other Mode is selected then
       tempMode = 0;                    // Default to "0"

    wiringPiSPISetupMode(this->SPIChannel, speed, tempMode);
        // Enable SPI interface for selected SPI channel, speed and mode
}
#else
//==================================================================================================
SPIDevice::SPIDevice() {
}
#endif

uint8_t SPIDevice::SPITransfer(uint8_t *pData, uint16_t size) {
/**************************************************************************************************
 * Transfers 8bit data from Master device (this device) and transfers to selected device
 * Function assumes that the Slave Select pin has already been pulled low, so upper class (which
 * will device the use of device, will need to pull selection pin low)
 * Receives a pointer to data to be transfered, and the number of bytes to transmit.
 * Will then put any data read back from the slave onto the same array
 *************************************************************************************************/
    if (pData == __null || size == 0)       // If no data has been requested to be set
        return -1;                          // return error

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the SPI interface has been enabled
    if ((this->SPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
        __HAL_SPI_ENABLE(this->SPIHandle);  // Enable the SPI interface

    while(size > 0) {                       // Whilst there is data to be transfered
        // Wait for the transfer buffer to be empty before putting data for transfer
        while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_TXE) == 0x00) {};
            // No timeout period has been specified - Can get stuck

        this->SPIHandle->Instance->DR = *pData; // Put data onto buffer for transfer

        // Wait for the transfer to complete, and data to be read back from SLAVE
        while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_RXNE) == 0x00) {};

        *pData = this->SPIHandle->Instance->DR; // Put data read from device back into array
        pData += sizeof(uint8_t);               // Increment pointer by the size of the data to be
                                                // transmitted
        size--;                                 // Decrement count
    }

    // Only exit function once transfer is complete -> detected by BUSY flag clearing
    while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_BSY) == 0x01) {};

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    wiringPiSPIDataRW(this->SPIChannel, pData, (int)size);
        // Using wiringPiSPI function, transfer data from RaspberryPi to selected device

#else
//==================================================================================================

#endif

    return 0;
}

SPIDevice::~SPIDevice()
{
    // TODO Auto-generated destructor stub
}

