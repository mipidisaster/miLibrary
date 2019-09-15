/**************************************************************************************************
 * @file        I2CPeriph.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        15 Sept 2019
 * @brief       Source file for the Generic I2C Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/

#include <FileIndex.h>
#include FilInd_I2CPe__HD

void I2CPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the I2CPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    this->Flt           = DevFlt::Initialised;      // Initialise the fault to "initialised"
    this->CommState     = CommLock::Free;           // Indicate bus is free

    this->curCount      = 0;                // Initialise the current packet size count
    this->curForm       = { 0 };            // Initialise the form to a blank entry

    this->curReqst      = Request::Nothing; // Initialise the current request state to 0
}

I2CPeriph::I2CPeriph(void) {
/**************************************************************************************************
 * Basic construction of I2C Periph Device
 *************************************************************************************************/
    this->popGenParam();                    // Populate generic class parameters

    this->I2C_Handle    = __null;           // Point to NULL
}

void I2CPeriph::create(I2C_HandleTypeDef *I2C_Handle, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * Creates a I2C class specific for the STM32 device.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired I2C
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->popGenParam();                    // Populate generic class parameters

    this->I2C_Handle    = I2C_Handle;       // Link input I2C handler to class.

    this->FormQueue.create(FormArray, FormSize);
}

I2CPeriph::I2CPeriph(I2C_HandleTypeDef *I2C_Handle, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * Creates a I2C class specific for the STM32 device.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired I2C
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->create(I2C_Handle, FormArray, FormSize);
}

uint8_t I2CPeriph::DRRead(void) {
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

void I2CPeriph::DRWrite(uint8_t data) {
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

uint8_t I2CPeriph::TransmitEmptyChk(void) {
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

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TXE)  != 0 ) || \
         (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TXIS) != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::TransmitComptChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit communication (if completed, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// I2C communication for STM32L has two "Transmission Complete" flags, one is generic, the other
// is used for Complete transmission for I2C Reload

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TC)   != 0 ) || \
         (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_TCR)  != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::ReceiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_RXNE) != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::BusNACKChk(void) {
/**************************************************************************************************
 * Check to see if peripheral has NOT acknowledge the packet (if NACK, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_AF)   != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::ClearNACK(void) {
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

uint8_t I2CPeriph::BusSTOPChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus has been "STOPPED" (if bus is STOP, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_STOPF) != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::Clear_STOP(void) {
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

uint8_t I2CPeriph::BusBusyChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_BUSY) != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::BusErroChk(void) {
/**************************************************************************************************
 * Check to see if there has been an incorrect START/STOP bit sent (if fault, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if ( (__HAL_I2C_GET_FLAG(this->I2C_Handle, I2C_FLAG_BERR) != 0 ) )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::ClearBusEr(void) {
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

uint8_t I2CPeriph::TransmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_TXI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::TransmitComptITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Complete Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_TCI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::ReceiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_RXI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::BusNACKITChk(void) {
/**************************************************************************************************
 * Check to see whether the Negative Acknowledge interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_NACKI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::BusSTOPITChk(void) {
/**************************************************************************************************
 * Check to see whether the STOP bus interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_STOPI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

uint8_t I2CPeriph::BusErrorITChk(void) {
/**************************************************************************************************
 * Check to see whether the Bus Error interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (__HAL_I2C_GET_IT_SOURCE(this->I2C_Handle, I2C_IT_ERRI) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::RequestTransfer(uint16_t devAddress, uint8_t size, CommMode mode, Request reqst) {
/**************************************************************************************************
 * Function will configure the device hardware/internal class parameters to setup a I2C
 * communication between this device (local) and the selected device (external). This communicate
 * can be either read/write, however the maximum number of packets which can be transmitted is
 * limited to 255 (1 byte). If more is required then this function will need to be called multiple
 * times with divisions of 255 to complete the communication.
 * The devAddress that is provided needs to be provided as a 11bit or 8bit address; so the last
 * bit which is the read/write bit will need to be set to blank; actual entry will be ignored
 * however.
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
    if (reqst != Request::Nothing)
        this->I2C_Handle->Instance->CR2 &= ~(I2C_CR2_RD_WRN);
    // Now the register has been cleared, so can now populate with the parameters for the new
    // transfer
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup Slave Address:
    this->I2C_Handle->Instance->CR2 |= ((uint32_t)devAddress & I2C_CR2_SADD);

    // Setup the size of data to be transmitted:
    this->I2C_Handle->Instance->CR2 |= (((uint32_t)size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);

    this->curCount   = size;        // Capture the number of packets to transmit

    // Setup the communication mode:
    if      (mode == CommMode::AutoEnd)                 // If mode is "AutoEnd"
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_AUTOEND_MODE);
        // Set the AutoEnd bit in register
    else if (mode == CommMode::Reload)                  // If mode is "Reload"
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_RELOAD_MODE);
        // Set the RELOAD bit in register
    // The other mode "I2C_SoftEnd", is set by clearing both of these bits. Which is done during
    // the register clear part.

    // Setup the Request mode
    if      (reqst == Request::START_WRITE) {           // If request is for START_WRITE
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_START);
        // just set START bit (write is done, by clearing the read bit)
        this->curReqst = reqst;     // Bring across the request mode
    }
    else if (reqst == Request::START_READ) {            // If request is for START_READ
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN);
        // Set the START bit, along with the Read bit
        this->curReqst = reqst;     // Bring across the request mode
    }
    else if (reqst == Request::STOP)                    // If request is for STOP
        this->I2C_Handle->Instance->CR2 |= (uint32_t)(I2C_CR2_STOP);
        // Set the STOP bit

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet

#else
//==================================================================================================

#endif

}

I2CPeriph::Form I2CPeriph::GenericForm(uint16_t devAddress, uint8_t size, CommMode mode,
                                          Request reqst,
                                          volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Generate a I2CForm request, based upon the generic information provided as input.
 *************************************************************************************************/
    I2CPeriph::Form RequestForm = { 0 };        // Generate the "I2CPeriph::Form" variable to
                                                // provide as output

    RequestForm.devAddress      = devAddress;   // Populate form with input data
    RequestForm.size            = size;         //
    RequestForm.Reqst           = reqst;        //
    RequestForm.Mode            = mode;         //

    // Indications used for source functionality to get status of requested communication
    RequestForm.Flt             = fltReturn;    // Populate return fault flag
    RequestForm.Cmplt           = cmpFlag;      // Populate complete communication indication

    return (RequestForm);
}

void I2CPeriph::FormW8bitArray(I2CPeriph::Form *RequestForm, uint8_t *pData) {
/**************************************************************************************************
 * Link input 8bit array pointer to the provided I2C Request Form.
 *************************************************************************************************/
    RequestForm->Buff       = pData;        // Pass data pointer to I2CForm
        // Indicate that data type is 8bit array.
}

void I2CPeriph::specificRequest(uint16_t devAddress, uint8_t size, uint8_t *pData,
                        CommMode mode, Request reqst,
                        volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Function used to populate the internal I2C Form stack, with the input requested communication.
 * Data comes from an array (pointer - pData)
 *************************************************************************************************/
    I2CPeriph::Form RequestForm = this->GenericForm(devAddress, size, mode, reqst,
                                                    fltReturn, cmpFlag);

    this->FormW8bitArray(&RequestForm, pData);


    this->FormQueue.InputWrite(RequestForm);    // Put request onto I2C Form Queue
}

uint8_t I2CPeriph::GetFormWriteData(Form *RequestForm) {
/**************************************************************************************************
 * Retrieve the next data point to write to external device from the selected I2C Request form
 *************************************************************************************************/
    uint8_t tempval = 0;        // Temporary variable to store data value

    tempval = *(RequestForm->Buff);             // Retrieve data from array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer

    return (tempval);       // Return value outside of function
}

void I2CPeriph::PutFormReadData(Form *RequestForm, uint8_t readdata) {
/**************************************************************************************************
 * Data read from the I2C external device is copied into the requested source location, as per
 * the I2C Request form
 *************************************************************************************************/
    *(RequestForm->Buff)    = readdata;         // Put data into array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer
}

I2CPeriph::DevFlt I2CPeriph::poleMasterTransmit(uint16_t devAddress, uint8_t *pdata,
                                                uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * send the requested amount of data to be written to the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    this->CommState = Communicating;        // Indicate bus is communicating

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    //this->Enable();                     // Ensure that the device has been enabled

    while(this->BusBusyChk() == 1) {}   // Wait until the bus is no longer busy

    this->RequestTransfer(devAddress, size, CommMode::AutoEnd, Request::START_WRITE);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    while(size != 0) {
        while(this->TransmitEmptyChk() == 0) {  // Whilst checking for Transmit Empty
            if (this->BusNACKChk() == 1) {      // If there is a NACK
                this->ClearNACK();              // Clear the NACK bit
                return (this->Flt = DevFlt::NACK);      // Fault status of "I2C_NACK"
            }

            if (this->BusErroChk() == 1) {      // If there has been a bus error
                this->ClearBusEr();             // Clear the Bus error bit
                return (this->Flt = DevFlt::BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
            }
        }
        this->DRWrite(*pdata);                  // Put data onto the TXDR for transmission
        pdata++;                                // Increment array
        size--;                                 // Decrease size
    }

    while(this->BusSTOPChk() == 0) {        // Wait on the STOP to be set
        if (this->BusNACKChk() == 1) {      // If there is a NACK
            this->ClearNACK();              // Clear the NACK bit
            return (this->Flt = DevFlt::NACK);          // Fault status of "I2C_NACK"
        }

        if (this->BusErroChk() == 1) {      // If there has been a bus error
            this->ClearBusEr();             // Clear the Bus error bit
            return (this->Flt = DevFlt::BUS_ERROR);     // Fault status of "I2C_BUS_ERROR"
        }
    }

    this->Clear_STOP();                     // Clear the STOP bit

    //this->Disable();                    // Ensure that the device has been disable

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet
#else
//==================================================================================================
// Device not supported

#endif

    // Indicate that the bus is free
    this->CommState = Free;             // Indicate bus is free

    return (this->Flt = DevFlt::None);      // No fault by this point, so return no fault
}

I2CPeriph::DevFlt I2CPeriph::poleMasterReceive(uint16_t devAddress, uint8_t *pdata,
                                               uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * read back the request amount of data from the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    this->CommState = Communicating;        // Indicate bus is communicating

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    //this->Enable();                     // Ensure that the device has been enabled

    while(this->BusBusyChk() == 1) {}   // Wait until the bus is no longer busy

    this->RequestTransfer(devAddress, size, CommMode::AutoEnd, Request::START_READ);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    while(size != 0) {
        while(this->ReceiveToReadChk() == 0) {  // Whilst checking for Receive buffer is full
            if (this->BusNACKChk() == 1) {      // If there is a NACK
                this->ClearNACK();              // Clear the NACK bit
                return (this->Flt = DevFlt::NACK);      // Fault status of "I2C_NACK"
            }

            if (this->BusErroChk() == 1) {      // If there has been a bus error
                this->ClearBusEr();             // Clear the Bus error bit
                return (this->Flt = DevFlt::BUS_ERROR); // Fault status of "I2C_BUS_ERROR"
            }
        }
        *pdata = this->DRRead();                // Read the data from RXDR into array
        pdata++;                                // Increment array
        size--;                                 // Decrease size
    }

    while(this->BusSTOPChk() == 0) {        // Wait on the STOP to be set
        if (this->BusNACKChk() == 1) {      // If there is a NACK
            this->ClearNACK();              // Clear the NACK bit
            return (this->Flt = DevFlt::NACK);          // Fault status of "I2C_NACK"
        }

        if (this->BusErroChk() == 1) {      // If there has been a bus error
            this->ClearBusEr();             // Clear the Bus error bit
            return (this->Flt = DevFlt::BUS_ERROR);     // Fault status of "I2C_BUS_ERROR"
        }
    }

    this->Clear_STOP();                     // Clear the STOP bit

    //this->Disable();                    // Ensure that the device has been disable

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Not populated yet
#else
//==================================================================================================
// Device not supported

#endif

    // Indicate that the bus is free
        this->CommState = Free;             // Indicate bus is free

    return (this->Flt = DevFlt::None);      // No fault by this point, so return no fault
}

I2CPeriph::DevFlt I2CPeriph::poleDeviceRdy(uint16_t devAddress) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * read back the request amount of data from the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    this->CommState = Communicating;        // Indicate bus is communicating

    //this->Enable();                     // Ensure that the device has been enabled

    RequestTransfer(devAddress, 0, CommMode::AutoEnd, Request::START_WRITE);

    while(this->BusSTOPChk() == 0) {        // Wait on the STOP to be set
        if (this->BusNACKChk() == 1) {      // If there is a NACK
            this->ClearNACK();              // Clear the NACK bit
            return (DevFlt::NACK);          // Fault status of "I2C_NACK"
        }

        if (this->BusErroChk() == 1) {      // If there has been a bus error
            this->ClearBusEr();             // Clear the Bus error bit
            return (DevFlt::BUS_ERROR);     // Fault status of "I2C_BUS_ERROR"
        }
    }

    this->Clear_STOP();                     // Clear the STOP bit

    //this->Disable();                    // Ensure that the device has been disable

    // Indicate that the bus is free
        this->CommState = Free;             // Indicate bus is free

    return(DevFlt::None);
}

void I2CPeriph::configTransmtIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_TXI);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_TXI); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::configTransCmIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Complete interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_TCI);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_TCI); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_RXI);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_RXI); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::configBusNACKIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus NACK interrupt
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                         // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_NACKI);    // Then enable the interrupt
    }
    else {                                                      // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_NACKI);   // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::configBusSTOPIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus STOP interrupt
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                         // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_STOPI);    // Then enable the interrupt
    }
    else {                                                      // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_STOPI);   // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::configBusErroIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus Error interrupt
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                         // If request is to enable
        __HAL_I2C_ENABLE_IT(this->I2C_Handle, I2C_IT_ERRI);     // Then enable the interrupt
    }
    else {                                                      // If request is to disable
        __HAL_I2C_DISABLE_IT(this->I2C_Handle, I2C_IT_ERRI);    // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void I2CPeriph::intMasterReq(uint16_t devAddress, uint8_t size, uint8_t *Buff,
                             CommMode mode, Request reqst,
                             volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new I2C communication.
 * This version of the I2C Form update functions, is expected to be only used for "Interrupt"
 * based communication, and therefore uses the "GenBuffer" handle input.
 *************************************************************************************************/
    // Put request into Form Queue
    this->specificRequest(devAddress,
                          size,
                          Buff,
                          mode, reqst,
                          fltReturn, cmpFlag
                         );

    // Trigger interrupt(s)
    this->I2CInterruptStart();
}

void I2CPeriph::I2CInterruptStart(void) {
/**************************************************************************************************
 * Function will be called to start off a new I2C communication if there is something in the
 * queue, and the bus is free.
 *************************************************************************************************/
    if ( (this->CommState == Free) && (this->FormQueue.State() != GenBuffer_Empty) ) {
        // If the I2C bus is free, and there is I2C request forms in the queue
        this->FormQueue.OutputRead( &(this->curForm) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(this->curForm.Flt) != I2CPeriph::DevFlt::None  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( this->FormQueue.State() == GenBuffer_Empty ) { // If buffer is empty, break out
                //this->Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            this->FormQueue.OutputRead( &(this->curForm) );     // Capture form request
        }

        this->CommState = Communicating;        // Lock I2C bus

        //this->Enable();

        this->RequestTransfer(  this->curForm.devAddress,
                                this->curForm.size,
                                this->curForm.Mode,
                                this->curForm.Reqst
                             );
            // Trigger communication as per Form request

        if (this->curReqst == Request::START_WRITE) {       // If this is a write request
            this->configTransmtIT(ITEnable);                // Then enable Transmit Empty buffer
        }                                                   // interrupt

        else if (this->curReqst == Request::START_READ) {   // If this is a read request
            this->configReceiveIT(ITEnable);                // Then enable Receive buffer full
        }                                                   // interrupt
    }
    else if ( (this->CommState == Free) && (this->FormQueue.State() == GenBuffer_Empty) ) {
        //this->Disable();
    }
}

void I2CPeriph::intReqFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form, to indicate how much data has been completed.
 * Indicate that the I2C Device is now free for any new communication
 * Disables the Receive/Transmit Interrupts
 *************************************************************************************************/
    *(this->curForm.Cmplt)  += (this->curForm.size - this->curCount);
    // Indicate how many data points have been transfered (curCount should be 0)

    // Indicate that I2C bus is now free, and disable any interrupts
    this->CommState = Free;

    this->configReceiveIT(ITDisable);       // Disable Receive buffer full interrupt
    this->configTransmtIT(ITDisable);       // Disable Transmit empty buffer interrupt
}

void I2CPeriph:: IRQEventHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the I2C events within the I2C class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occured, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:
 *      Transmit Complete Flag
 *          - This functionality is not supported, however this status should be used if the
 *            "Auto" communication has not been selected. Such that when the current pack of 255
 *            bytes to transmit has been done, completed then the same communication can continue
 *            with a new set of data.
 *            For communicating packets of data > 255bytes.
 *            Note - Will be cleared by hardware, when either a START or STOP has been triggered.
 *
 *      Transmit Buffer Empty
 *          - Used to pull out more data from the current Request form, and put onto the Transmit
 *            hardware buffer.
 *            Note - Writing to the hardware will clear this status bit.
 *
 *      Receive Buffer full (not empty)
 *          - Similar to the Transmit Buffer Empty interrupt, this interrupt will be used to read
 *            from the hardware the current read value. This will then be passed to the requested
 *            data location (as per Request form).
 *
 *      I2C Stop has been set
 *          - If a STOP has been detected, then current Request form has been completed.
 *            Will then indicate how much data has been transmitted, and look to see if there is
 *            another other requests. If none, then communication interrupts are disabled.
 *
 *      I2C data has not been acknowledged (NACK)
 *          - If a NACK is detected during the communication with the external device. Then the
 *            current Request Form is binned. A fault is provided to the requested source
 *            location, and the return complete indication is setup to show how many bytes had
 *            been transmitted prior to NACK.
 *            Source functional will then need to determine what to do in this situation.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    uint8_t tempDR = 0;

    if ( (this->TransmitComptChk() & this->TransmitComptITChk()) == 0x01) {
        // If Transmit Complete triggered
        // Not really supported yet!
    }

    if ( (this->TransmitEmptyChk() & this->TransmitEmptyITChk()) == 0x01) {
        // If Transmit Buffer Empty triggered
        // Only update the transmit hardware buffer, if the class global count is not zero
        if (this->curCount == 0)    // If count is equal to zero
            tempDR  = 0x00;         // Populate with zeroes (default data)

        else {
            tempDR  = GetFormWriteData( &(this->curForm) );
            // Retrieve next data point from the Request SPI Form, and put onto hardware queue

            this->curCount--;       // Decrement the class global current count
        }
        // Get data from form, and put into hardware buffer

        this->DRWrite(tempDR);
    }

    if ( (this->ReceiveToReadChk() & this->ReceiveToReadITChk()) == 0x01) {
        // If Receive Buffer full triggered
        tempDR  = this->DRRead();
        // Retrieve data from the hardware

        // Only when the class global count is not zero. Put the read data into the requested data
        // location as per I2C Request Form
        if (this->curCount != 0) {
            this->PutFormReadData( &(this->curForm) , tempDR );

            this->curCount--;       // Decrement the class global current count
        }
    }

    if ( (this->BusSTOPChk() & this->BusSTOPITChk()) == 0x01) {     // If I2C Stop triggered
        this->Clear_STOP();                                         // Clear the STOP status
        this->ClearNACK();                                          // Clear the NACK status

        // Flush the contents of the Transmit buffer
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          __HAL_I2C_CLEAR_FLAG(this->I2C_Handle, I2C_FLAG_TXE);

        this->intReqFormCmplt();                                    // Complete the current
                                                                    // request form
        this->I2CInterruptStart();                                  // Check if any new requests
                                                                    // remain
    }

    if ( (this->BusNACKChk() & this->BusNACKITChk()) == 0x01) {     // If I2C NACK received
        this->ClearNACK();                                          // Clear the NACK status
        *(this->curForm.Flt)    = DevFlt::NACK;                     // Set requested fault flag

        // Flush the contents of the Transmit buffer
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          __HAL_I2C_CLEAR_FLAG(this->I2C_Handle, I2C_FLAG_TXE);

        this->intReqFormCmplt();                                    // Complete the current
                                                                    // request form
        this->I2CInterruptStart();                                  // Check if any new requests
                                                                    // remain
    }
}

void I2CPeriph:: IRQErrorHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the I2C errors within the I2C class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occurred, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:
 *      Bus Error
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    if ( (this->BusErroChk() & this->BusErrorITChk()) == 0x01) {    // If Bus Error
                                                                    // triggered
        this->ClearBusEr();                                         // Clear the Bus Error
        *(this->curForm.Flt)    = DevFlt::BUS_ERROR;                // Set requested fault flag
    }
}

I2CPeriph::~I2CPeriph()
{
    // TODO Auto-generated destructor stub
}

