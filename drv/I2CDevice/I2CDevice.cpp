/**************************************************************************************************
 * @file        I2CDevice.cpp
 * @author      Thomas
 * @version     V0.2
 * @date        27 Oct 2018
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/

#include "FileIndex.h"
#include FilInd_I2CDe__HD

uint8_t I2CDevice::DRRead(void) {
/**************************************************************************************************
 * Read from the I2C hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    return ((uint8_t) this->I2C_Handle->Instance->RXDR);    // Read from the RX Data Register

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CDevice::DRWrite(uint8_t data) {
/**************************************************************************************************
 * Write to the I2C hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    this->I2C_Handle->Instance->TXDR = data;        // Put data onto the TX Data Register

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::TransmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
//  The STM32L device has two bits which can be used to determine if the Transmit buffer is empty,
//  they are TXE = Transmit Buffer empty bit, and the TXIE = Transmit Buffer status
//     TXIE is used to trigger the interrupt
//  Both are used within this function call.

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TXE)  == SET) || \
         (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TXIS) == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::TransmitComptChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit communication (if completed, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// I2C communication for STM32L has two "Transmission Complete" flags, one is generic, the other
// is used for Complete transmission for I2C Reload

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TC)   == SET) || \
         (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TCR)  == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::ReceiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_RXNE) == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::BusNACKChk(void) {
/**************************************************************************************************
 * Check to see if peripheral has NOT acknowledge the packet (if NACK, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_AF)   == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CDevice::ClearNACK(void) {
/**************************************************************************************************
 * Clear the NACK bit in the status register
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    __HAL_I2C_CLEAR_FLAG(this->I2C_Handle, I2C_FLAG_AF);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::BusBusyChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_BUSY) == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::BusSTOPChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus has been "STOPPED" (if bus is STOP, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_STOPF) == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CDevice::BusErroChk(void) {
/**************************************************************************************************
 * Check to see if there has been an incorrect START/STOP bit sent (if fault, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_BERR) == SET) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CDevice::ClearBusEr(void) {
/**************************************************************************************************
 * Clear the Bus error status bit
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    __HAL_I2C_CLEAR_FLAG(this->I2C_Handle, I2C_FLAG_BERR);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CDevice::Clear_STOP(void) {
/**************************************************************************************************
 * Clear the Bus STOP bit status
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    __HAL_I2C_CLEAR_FLAG(this->I2C_Handle, I2C_FLAG_STOPF);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CDevice::RequestTransfer(uint16_t devAddress, uint8_t size, _I2CCommMode mode,
                                _I2CRequest reqst) {
/**************************************************************************************************
 * Function will configure the device hardware/internal class parameters to setup a I2C
 * communication between this device (Master) and the selected device (Slave). This communicate
 * can be either read/write, however the maximum number of packets which can be transmitted is
 * limited to 255 (1 byte). If more is required then this function will need to be called multiple
 * times with divisions of 255 to complete the communication.
 * The devAddress that is provided needs to be provided as a 11bit or 8bit address; so the last
 * bit which is the read/write bit will need to be set to zero.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// Not populated yet

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// For the STM32Lx device the configuration of I2C for data transmission/reception is straight
// forward, all that is required is to populate the CR2 hardware register with Device Address
// Read/Write status, and populate the number of packets to send

    // Clear the CR2 register of generic conditions:
    //          Slave Address       (SADD)
    //          Bytes to transmit   (NBYTES) (limited to 255)
    //          Reload/AutoEnd      (RELOAD/AUTOEND)
    //          START/STOP          (START/STOP)
    this->I2C_Handle->Instance->CR2 &= ~((I2C_CR2_SADD   | \
                                          I2C_CR2_NBYTES | \
                                          I2C_CR2_RELOAD | I2C_CR2_AUTOEND | \
                                          I2C_CR2_START  | I2C_CR2_STOP));
    // If the request is anything other than NOTHING, then clear the Read/~Write bit
    if (reqst != I2C_Nothing)
        this->I2C_Handle->Instance->CR2 &= ~(I2C_CR2_RD_WRN);

    // Now the register has been cleared, so can now populate with the parameters for the new
    // transfer
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup Slave Address:
    this->I2C_Handle->Instance->CR2 |= ((uint32_t)devAddress & I2C_CR2_SADD);

    // Setup the size of data to be transmitted:
    this->I2C_Handle->Instance->CR2 |= (((uint32_t)size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);

    this->curSize   = size;         // Capture the number of packets to transmit

    // Setup the communication mode:
    if      (mode == I2C_AutoEnd)                                   // If mode is "AutoEnd"
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_AUTOEND_MODE);
        // Set the AutoEnd bit in register
    else if (mode == I2C_Reload)                                    // If mode is "Reload"
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_RELOAD_MODE);
        // Set the RELOAD bit in register
    // The other mode "I2C_SoftEnd", is set by clearing both of these bits. Which is done during
    // the register clear part.

    // Setup the Request mode
    if      (reqst == I2C_START_WRITE) {                        // If request is for START_WRITE
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_START);
        // just set START bit (write is done, by clearing the read bit)
        this->curReqst = reqst;     // Bring across the request mode
    }
    else if (reqst == I2C_START_READ) {                         // If request is for START_READ
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN);
        // Set the START bit, along with the Read bit
        this->curReqst = reqst;     // Bring across the request mode
    }
    else if (reqst == I2C_STOP)                                 // If request is for STOP
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_STOP);
        // Set the STOP bit

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet

#else
//==================================================================================================

#endif

}

I2CDevice::I2CDevice(I2C_HandleTypeDef *I2C_Handle) {
/**************************************************************************************************
 * Creates a I2C class specific for the STM32F device.
 *
 * As the STM32CubeMX alread y pre-generates the setting up and configuring of the desired I2C
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->I2C_Handle    = I2C_Handle;      // Copy data into class
    this->Flt           = I2C_Initialised; // Initialise the fault to "initialised"

    this->curSize       = 0;                // Initialise the current packet size to 0
    this->curReqst      = I2C_Nothing;      // Initialise the current request state to 0

}

_I2CDevFlt I2CDevice::poleMasterTransmit(uint16_t devAddress, uint8_t *pdata, uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * send the requested amount of data to be written to the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    while(this->BusBusyChk() == 1) {}   // Wait until the bus is no longer busy

    this->RequestTransfer(devAddress, size, I2C_AutoEnd, I2C_START_WRITE);

    while(size != 0) {
        while(this->TransmitEmptyChk() == 0) {  // Whilst checking for Transmit Empty
            if (this->BusNACKChk() == 1) {      // If there is a NACK
                this->ClearNACK();              // Clear the NACK bit
                return (this->Flt = I2C_NACK);  // Fault status of "I2C_NACK"
            }

            if (this->BusErroChk() == 1) {      // If there has been a bus error
                this->ClearBusEr();             // Clear the Bus error bit
                return (this->Flt = I2C_BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
            }
        }
        this->DRWrite(*pdata);                  // Put data onto the TXDR for transmission
        pdata++;                                // Increment array
        size--;                                 // Decrease size
    }

    while(this->BusSTOPChk() == 0) {        // Wait on the STOP to be set
        if (this->BusNACKChk() == 1) {      // If there is a NACK
            this->ClearNACK();              // Clear the NACK bit
            return (this->Flt = I2C_NACK);  // Fault status of "I2C_NACK"
        }

        if (this->BusErroChk() == 1) {      // If there has been a bus error
            this->ClearBusEr();             // Clear the Bus error bit
            return (this->Flt = I2C_BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
        }
    }

    this->Clear_STOP();                     // Clear the STOP bit

    return (this->Flt = I2C_NoFault);       // No fault by this point, so return no fault

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet
#else
//==================================================================================================
// Device not supported

#endif
}

_I2CDevFlt I2CDevice::poleMasterReceive(uint16_t devAddress, uint8_t *pdata, uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * read back the request amount of data from the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    while(this->BusBusyChk() == 1) {}   // Wait until the bus is no longer busy

    this->RequestTransfer(devAddress, size, I2C_AutoEnd, I2C_START_READ);

    while(size != 0) {
        while(this->ReceiveToReadChk() == 0) {  // Whilst checking for Receive buffer is full
            if (this->BusNACKChk() == 1) {      // If there is a NACK
                this->ClearNACK();              // Clear the NACK bit
                return (this->Flt = I2C_NACK);  // Fault status of "I2C_NACK"
            }

            if (this->BusErroChk() == 1) {      // If there has been a bus error
                this->ClearBusEr();             // Clear the Bus error bit
                return (this->Flt = I2C_BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
            }
        }
        *pdata = this->DRRead();                // Read the data from RXDR into array
        pdata++;                                // Increment array
        size--;                                 // Decrease size
    }

    while(this->BusSTOPChk() == 0) {        // Wait on the STOP to be set
        if (this->BusNACKChk() == 1) {      // If there is a NACK
            this->ClearNACK();              // Clear the NACK bit
            return (this->Flt = I2C_NACK);  // Fault status of "I2C_NACK"
        }

        if (this->BusErroChk() == 1) {      // If there has been a bus error
            this->ClearBusEr();             // Clear the Bus error bit
            return (this->Flt = I2C_BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
        }
    }

    this->Clear_STOP();                     // Clear the STOP bit

    return (this->Flt = I2C_NoFault);       // No fault by this point, so return no fault

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet
#else
//==================================================================================================
// Device not supported

#endif
}



I2CDevice::~I2CDevice()
{
    // TODO Auto-generated destructor stub
}

