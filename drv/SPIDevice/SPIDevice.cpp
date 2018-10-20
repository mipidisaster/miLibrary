/**************************************************************************************************
 * @file        SPIDevice.cpp
 * @author      Thomas
 * @version     V1.2
 * @date        09 Oct 2018
 * @brief       Source file for the Generic SPIDevice Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_SPIDe__HD

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
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

uint8_t SPIDevice::SPIRWTransfer(uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Transfers 8bit data from Master device (this device) and transfers to selected device.
 * Function assumes that the Slave Select pin has already been pulled low, so upper class (which
 * will need to have selected the device).
 * It is provided a pointer to the data to written, and the pointer to the data to be read - along
 * the size.
 *************************************************************************************************/
    if (wData == __null || rData == __null || size == 0)    // If no data has been requested
        return -1;                                          // to be set return error

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the SPI interface has been enabled
    if ((this->SPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
        __HAL_SPI_ENABLE(this->SPIHandle);  // Enable the SPI interface

    while(size > 0) {                       // Whilst there is data to be transfered
        // Wait for the transfer buffer to be empty before putting data for transfer
        while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_TXE) == 0x00) {};
            // No timeout period has been specified - Can get stuck

        this->SPIHandle->Instance->DR = *wData; // Put data onto buffer for transfer

        // Wait for the transfer to complete, and data to be read back from SLAVE
        while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_RXNE) == 0x00) {};

        *rData = this->SPIHandle->Instance->DR; // Put data read from device back into array
        wData += sizeof(uint8_t);               // Increment pointer for write array by the size of
                                                // the data type.
        rData += sizeof(uint8_t);               // Increment pointer for read array by the size of
                                                // the data type.
        size--;                                 // Decrement count
    }

    // Only exit function once transfer is complete -> detected by BUSY flag clearing
    while(__HAL_SPI_GET_FLAG(this->SPIHandle, SPI_FLAG_BSY) == 0x01) {};

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    // Ensure that the SPI interface has been enabled
    if ((this->SPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
        __HAL_SPI_ENABLE(this->SPIHandle);  // Enable the SPI interface

    HAL_SPI_TransmitReceive(this->SPIHandle, wData, rData, size, 100);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    uint16_t i = 0;

    for (i = 0; i != size; i++)         // Cycle through the data to be written
        rData[i] = wData[i];            // and copy into the read data


    wiringPiSPIDataRW(this->SPIChannel, rData, (int)size);
        // Using wiringPiSPI function, transfer data from RaspberryPi to selected device

#else
//==================================================================================================

#endif

    return 0;
}

/**************************************************************************************************
 * SPITransfer is an overloaded function, so therefore has multiple uses
 *  The first   is the "default", where only the data and size is provided
 *  The second  is provided the GPIO for Chip Select, which will be pulled low prior to transfer
 *              and then pushed back high after
 *************************************************************************************************/
uint8_t SPIDevice::SPITransfer(uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Default use of SPITransfer, no slave selection is done within this function
 *************************************************************************************************/
    return(this->SPIRWTransfer(wData, rData, size));    // Use private function to transfer data
}

uint8_t SPIDevice::SPITransfer(GPIO *ChipSelect, uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Slave selection is done within this function, where GPIO class (ChipSelect) is pulled LOW
 * prior to transmission, and then pushed HIGH after transmission
 *************************************************************************************************/
    uint8_t returnvalue = 0;                        // Variable to store output of SPIRWTransfer

    ChipSelect->setValue(GPIO_LOW);                 // Select the SLAVE (pull Chip Select LOW)

    returnvalue = this->SPIRWTransfer(wData, rData, size);
        // Use private function to transfer data

    ChipSelect->setValue(GPIO_HIGH);                // De-select the SLAVE (push Chip Select HIGH)

    return(returnvalue);                            // Return providing the output of transfer
}

uint8_t SPIDevice::SPITransfer(DeMux *DeMuxCS, uint8_t CSNum,
                               uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Slave selection is done within this function, where the DeMux class (DeMuxCS) has been provided
 * along with the selection number (CSNum) for the SPI Slave device that transmission is required
 * for.
 * This will update the Demultiplexor to the desired selection number, then enable the DeMux.
 * However in the hardware the SPI Chip Select hardware pin will need to have been connected to
 * at least one of the Low Enable pins (otherwise it will not work).
 * This specific pin will not have been provided to the DeMux class, as is managed by the SPI
 * peripheral.
 * Once transmission has finished, the Demultiplexor will be disabled
 *************************************************************************************************/
    uint8_t returnvalue = 0;                        // Variable to store output of SPIRWTransfer

    DeMuxCS->updateselection(CSNum);                // Setup Demultiplexor for SPI Slave
                                                    // Chip Select
    DeMuxCS->enable();                              // Enable the Demultiplexor
            // This expects that at least 1 of the EnableLow pins has been linked to the
            // SPI hardware CS signal (will not has been allocated to the DeMuxCS entry)

    returnvalue = this->SPIRWTransfer(wData, rData, size);
        // Use private function to transfer data

    DeMuxCS->disable();                             // Disable the Demultiplexor

    return(returnvalue);                            // Return providing the output of transfer
}

SPIDevice::~SPIDevice()
{
    // TODO Auto-generated destructor stub
}

