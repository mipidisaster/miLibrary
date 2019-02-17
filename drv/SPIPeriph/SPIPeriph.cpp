/**************************************************************************************************
 * @file        SPIPeriph.cpp
 * @author      Thomas
 * @version     V2.2
 * @date        14 Nov 2018
 * @brief       Source file for the Generic SPIPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/

#include "FileIndex.h"
#include FilInd_SPIPe__HD

void SPIPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the SPIPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    this->Flt           = DevFlt::Initialised;      // Initialise the fault to "initialised"
    this->CommState     = CommLock::Free;           // Indicate bus is free

    this->curCount      = 0;                // Initialise the current packet size count

    this->curForm       = { 0 };            // Initialise the form to a blank entry
}

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
SPIPeriph::SPIPeriph(void) {
/**************************************************************************************************
 * Basic construction of SPI Periph Device
 *************************************************************************************************/
    this->SPI_Handle    = __null;           // Point to NULL
    this->Mode          = MODE0;            // Initialise to MODE0

    this->popGenParam();                    // Populate generic class parameters
}

void SPIPeriph::create(SPI_HandleTypeDef *SPIHandle, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for the STM32 device
 * Receives the address of the SPI Handle of device - generated from cubeMX
 * Will then determine which mode has been selected, based upon the states of the registers
 *************************************************************************************************/
    this->popGenParam();                    // Populate generic class parameters

    this->SPI_Handle        = SPIHandle;    // copy handle across into class

    this->FormQueue.create(FormArray, FormSize);

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

    this->Enable();                     // Enable the SPI device
}

SPIPeriph::SPIPeriph(SPI_HandleTypeDef *SPIHandle, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for the STM32 device
 * Receives the address of the SPI Handle of device - generated from cubeMX
 * Will then determine which mode has been selected, based upon the states of the registers
 *************************************************************************************************/
    this->create(SPIHandle, FormArray, FormSize);
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
void SPIPeriph::create(int channel, int speed, _SPIMode Mode) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for RaspberryPi
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    this->popGenParam();                // Populate generic class parameters

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


SPIPeriph::SPIPeriph(int channel, int speed, _SPIMode Mode) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for RaspberryPi
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    this->create(channel, speed, Mode);
}
#else
//==================================================================================================
SPIPeriph::SPIPeriph() {
}
#endif

void SPIPeriph::Enable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    __HAL_SPI_ENABLE(this->SPI_Handle);     // Enable the SPI interface

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    __HAL_SPI_ENABLE(this->SPI_Handle);     // Enable the SPI interface

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    // Done in initial call.

#else
//==================================================================================================

#endif
}

void SPIPeriph::Disable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    __HAL_SPI_DISABLE(this->SPI_Handle);    // Disable the SPI interface

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    __HAL_SPI_DISABLE(this->SPI_Handle);    // Disable the SPI interface

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    // Done in initial call.

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::DRRead(void) {
/**************************************************************************************************
 * Read from the SPI hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    return ((uint8_t) this->SPI_Handle->Instance->DR);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// STM32L4 uses a RXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)

    return( (uint8_t) this->SPI_Handle->Instance->DR );

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return 0;

#else
//==================================================================================================

#endif
}

void SPIPeriph::DRWrite(uint8_t data) {
/**************************************************************************************************
 * Write to the SPI hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    this->SPI_Handle->Instance->DR = data;

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// STM32L4 uses a TXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)
// To ensure only 8bits is transmitted, need to ensure that we cast the Data Register (DR) to
// unsigned 8bits, hence the initial casing

    *(uint8_t *)&this->SPI_Handle->Instance->DR = data;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
#endif
}

uint8_t SPIPeriph::TransmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_TXE) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_TXE) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::ReceiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_RXNE) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_RXNE) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::BusBusyChk(void) {
/**************************************************************************************************
 * Check to see if the SPI bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_BSY) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_BSY) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::BusOverRunChk(void) {
/**************************************************************************************************
 * Check to see if a Bus Over run has occurred (if over run, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_OVR) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_OVR) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//==================================================================================================

#endif
}

void SPIPeriph::ClearBusOvrRun(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Overrun fault state
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// To clear the Bus Over run status form STM32F, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(this->SPI_Handle);      // Utilise the existing MACRO

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// To clear the Bus Over run status form STM32L, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(this->SPI_Handle);      // Utilise the existing MACRO


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::BusModeFltChk(void) {
/**************************************************************************************************
 * Check to see if a Bus mode fault has occurred (if mode fault, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_MODF) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_FLAG(this->SPI_Handle, SPI_FLAG_MODF) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

void SPIPeriph::ClearBusModeFlt(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Mode fault state
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(this->SPI_Handle);     // Utilise the existing MACRO

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(this->SPI_Handle);     // Utilise the existing MACRO


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::TransmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_TXE) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_TXE) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::ReceiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_RXNE) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_RXNE) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

uint8_t SPIPeriph::BusErrorITChk(void) {
/**************************************************************************************************
 * Check to see whether the Bus Error interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_ERR) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(this->SPI_Handle, SPI_IT_ERR) == SET )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//==================================================================================================

#endif
}

SPIPeriph::CSHandle SPIPeriph::HardwareCS(void) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via the Hardware Chip Select (no software interaction required)
 *************************************************************************************************/
    CSHandle TempStruct = { 0 };                        // Initialise structure

    TempStruct.Type  = CSHandle::Hardware_Managed;      // Indicate that SPI device is hardware
                                                        // managed
    return (TempStruct);                                // Return build structure
}

SPIPeriph::CSHandle SPIPeriph::SoftwareGPIO(GPIO *CS) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via software selection of input GPIO pin.
 *************************************************************************************************/
    CSHandle TempStruct = { 0 };                        // Initialise structure

    TempStruct.GPIO_CS  = CS;                           // Link input Chip Select to struct
    TempStruct.Type  = CSHandle::Software_GPIO;         // Indicate that SPI device is software
                                                        // managed
    return (TempStruct);                                // Return build structure
}

void SPIPeriph::ChipSelectHandle(CSHandle selection, CSSelection Mode) {
/**************************************************************************************************
 * With the provided CSHandle struct, and the desired Mode (Select/Deselect), function will then
 * do the necessary work to enable the external SPI device for communication.
 *************************************************************************************************/
    if (selection.Type == CSHandle::Software_GPIO) {        // If the CSHandle indicates Software
                                                            // management, then...
        if (Mode == Select)                                 // If Mode is "Select"
            selection.GPIO_CS->setValue(GPIO::LOW);         // Pull the GPIO "LOW"

        else                                                // If Mode is "DeSelect"
            selection.GPIO_CS->setValue(GPIO::HIGH);        // Pull the GPIO "HIGH"
    }
    // Hardware Managed doesn't require any software interaction. Therefore doesn't do anything
    // is selected.
}

SPIPeriph::Form SPIPeriph::GenericForm(CSHandle devLoc, uint8_t size,
                                       volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Generate a SPIForm request, based upon the generic information provided as input.
 *************************************************************************************************/
    SPIPeriph::Form RequestForm = { 0 };        // Generate the "SPIPeriph::Form" variable to
                                                // provide as output

    RequestForm.devLoc          = devLoc;       // Populate form with input data
    RequestForm.size            = size;         //

    // Indications used for source functionality to get status of requested communication
    RequestForm.Flt             = fltReturn;    // Populate return fault flag
    RequestForm.Cmplt           = cmpFlag;      // Populate complete communication indication

    return (RequestForm);
}

void SPIPeriph::FormW8bitArray(Form *RequestForm, uint8_t *TxData, uint8_t *RxData) {
/**************************************************************************************************
 * Link input 8bit array pointer(s) to the provided SPI Request Form.
 *************************************************************************************************/
    RequestForm->TxBuff.ptr8bit     = TxData;           // Pass Transmit Buffer to SPIForm
    RequestForm->TxBufType          = Form::Array8bit;  // Indicate that data type is 8bit array

    RequestForm->RxBuff.ptr8bit     = TxData;           // Pass Transmit Buffer to SPIForm
    RequestForm->RxBufType          = Form::Array8bit;  // Indicate that data type is 8bit array
}

void SPIPeriph::FormWGenBuffer(Form *RequestForm, GenBuffer<uint8_t> *TxBuff,
                                                  GenBuffer<uint8_t> *RxBuff) {
/**************************************************************************************************
 * Link input GenBuffer pointer(s) to the provided SPI Request Form.
 *************************************************************************************************/
    RequestForm->TxBuff.ptrGenBuf   = TxBuff;           // Pass Transmit Buffer to SPIForm
    RequestForm->TxBufType          = Form::GenBuffer8bit;
    // Indicate that data type is an 8bit GenBuffer

    RequestForm->RxBuff.ptrGenBuf   = RxBuff;           // Pass Transmit Buffer to SPIForm
    RequestForm->RxBufType          = Form::GenBuffer8bit;
    // Indicate that data type is an 8bit GenBuffer
}

void SPIPeriph::FlushFormWritedData(Form *RequestForm, uint8_t count) {
/**************************************************************************************************
 * Function will receive in the input request form, as well as the amount of data that has already
 * been written (so if zero, all data has been written).
 * Then depending upon if the source data for the write is of the "GenBuffer" type, it will Erase
 * any remaining data within the write buffer - so as to maintain consistency.
 *************************************************************************************************/
    if (RequestForm->TxBufType == Form::GenBuffer8bit) {    // Check data is "GenBuffer"
        RequestForm->TxBuff.ptrGenBuf->ReadErase((RequestForm->size - count));
        // Then Erase the Transmit Buffer (write)
    }
}

uint8_t SPIPeriph::GetFormWriteData(Form *RequestForm) {
/**************************************************************************************************
 * Retrieve the next data point to write to external device from the selected SPI Request form
 *************************************************************************************************/
    uint8_t tempval = 0;        // Temporary variable to store data value

    if      (RequestForm->TxBufType == Form::Array8bit) {   // If data type is 8bit array
        tempval = *(RequestForm->TxBuff.ptr8bit);           // Retrieve data from array
        RequestForm->TxBuff.ptr8bit++;                      // Increment array pointer
    }
    else if (RequestForm->TxBufType == Form::GenBuffer8bit) // If data is GenBuffer
        RequestForm->TxBuff.ptrGenBuf->OutputRead(&tempval);// Retrieve data from buffer

    return (tempval);       // Return value outside of function
}

void SPIPeriph::PutFormReadData(Form *RequestForm, uint8_t readdata) {
/**************************************************************************************************
 * Data read from the SPI external device is copied into the requested source location, as per
 * the SPI Request form
 *************************************************************************************************/
    if      (RequestForm->RxBufType == Form::Array8bit) {   // If data type is 8bit array
        *(RequestForm->RxBuff.ptr8bit) = readdata;          // Put data into array
        RequestForm->RxBuff.ptr8bit++;                      // Increment array pointer
    }
    else if (RequestForm->RxBufType == Form::GenBuffer8bit) // If data is GenBuffer
        RequestForm->RxBuff.ptrGenBuf->InputWrite(readdata);// Put data into buffer
}

SPIPeriph::DevFlt SPIPeriph::poleMasterTransfer(uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the external device; selected by the Chip Select
 * Pin. The Master being this local device.
 *   This is an OVERLOADED function, and forms the bases of the poling transmission for SPI. This
 *   version doesn't pull down any pins within software. Relies upon a hardware managed CS.
 *************************************************************************************************/
    if (wData == __null || rData == __null || size == 0)    // If no data has been requested
        return ( this->Flt = DevFlt::DataSize );            // to be set return error

    // Indicate that the bus is not free
    this->CommState = Communicating;        // Indicate bus is communicating

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    this->Enable();                     // Ensure that the device has been enabled

    SET_BIT(this->SPI_Handle->Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);
        // Ensure that the RXNE bit is set when the Receive buffer is 1/4 full (i.e. 8bits is
        // within)

    while (size != 0) {
        // Wait for the transfer buffer to be empty before putting data for transfer, whilst
        // waiting, check for any faults
        while(this->TransmitEmptyChk() == 0) {
            if (this->BusOverRunChk() == 1) {   // Any Bus Over runs errors
                this->ClearBusOvrRun();         // Clear the Bus Overrun fault
                return ( this->Flt = DevFlt::Overrun );     // Indicate fault, and exit
            }

            if (this->BusModeFltChk() == 1) {   // Any Bus Mode faults detected
                this->ClearBusModeFlt();        // Clear the Bus Mode fault
                return ( this->Flt = DevFlt::ModeFault );   // Indicate fault, and exit
            }
        };
            // No timeout period has been specified - Can get stuck

        this->DRWrite(*wData);                              // Put data onto buffer for transfer

        // Wait for the transfer to complete, and data to be read back from SLAVE, whilst waiting
        // check for any faults
        while(this->ReceiveToReadChk() == 0) {
            if (this->BusOverRunChk() == 1) {   // Any Bus Over runs errors
                this->ClearBusOvrRun();         // Clear the Bus Overrun fault
                return ( this->Flt = DevFlt::Overrun );     // Indicate fault, and exit
            }

            if (this->BusModeFltChk() == 1) {   // Any Bus Mode faults detected
                this->ClearBusModeFlt();        // Clear the Bus Mode fault
                return ( this->Flt = DevFlt::ModeFault );   // Indicate fault, and exit
            }
        };

        *rData = this->DRRead();                // Put data read from device back into array
        wData += sizeof(uint8_t);               // Increment pointer for write array by the size
                                                // of the data type.
        rData += sizeof(uint8_t);               // Increment pointer for read array by the size of
                                                // the data type.
        size--;                                 // Decrement count
    }

    // Only exit function once transfer is complete -> detected by BUSY flag clearing
    while(this->BusBusyChk() == 1) {};

    this->Disable();                    // Ensure that the device has been disable

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

    // Indicate that the bus is free
    this->CommState = Free;             // Indicate bus is free

    return ( this->Flt = DevFlt::None );
}


SPIPeriph::DevFlt SPIPeriph::poleMasterTransfer(GPIO *ChipSelect,
                                        uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Second version of the OVERLOADED function.
 * This version utilises the basic version, however prior to calling this, pulls down the input
 * GPIO pin.
 *************************************************************************************************/
    DevFlt returnvalue = DevFlt::Initialised;       // Variable to store output of RWTransfer

    this->ChipSelectHandle(SoftwareGPIO(ChipSelect), Select);

    returnvalue = this->poleMasterTransfer(wData, rData, size);
        // Use private function to transfer data

    this->ChipSelectHandle(SoftwareGPIO(ChipSelect), Deselect);

    return(returnvalue);                            // Return providing the output of transfer
}

SPIPeriph::DevFlt SPIPeriph::poleMasterTransfer(DeMux *DeMuxCS, uint8_t CSNum,
                               uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Third version of the OVERLOADED function.
 * This version is able to select the device via the DeMux class (DeMuxCS), along with the
 * selection number for the device (CSNum).
 * This will update the Demultiplexor to the desired selection number, then enable the DeMux.
 * However in the hardware the SPI Chip Select hardware pin will need to have been connected to
 * at least one of the Low Enable pins (otherwise it will not work).
 * This specific pin will not have been provided to the DeMux class, as is managed by the SPI
 * peripheral.
 * Once transmission has finished, the Demultiplexor will be disabled
 *************************************************************************************************/
    DevFlt returnvalue = DevFlt::Initialised;       // Variable to store output of RWTransfer

    DeMuxCS->updateselection(CSNum);                // Setup Demultiplexor for SPI Slave
                                                    // Chip Select
    DeMuxCS->enable();                              // Enable the Demultiplexor
            // This expects that at least 1 of the EnableLow pins has been linked to the
            // SPI hardware CS signal (will not has been allocated to the DeMuxCS entry)

    returnvalue = this->poleMasterTransfer(wData, rData, size);
        // Use private function to transfer data

    DeMuxCS->disable();                             // Disable the Demultiplexor

    return(returnvalue);                            // Return providing the output of transfer
}

void SPIPeriph::configTransmtIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_SPI_ENABLE_IT(this->SPI_Handle, SPI_IT_TXE);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(this->SPI_Handle, SPI_IT_TXE); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void SPIPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_SPI_ENABLE_IT(this->SPI_Handle, SPI_IT_RXNE); // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(this->SPI_Handle, SPI_IT_RXNE);// Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void SPIPeriph::configBusErroIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus Error interrupt
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================

    if (intr == InterState::ITEnable) {                     // If request is to enable
        __HAL_SPI_ENABLE_IT(this->SPI_Handle, SPI_IT_ERR);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(this->SPI_Handle, SPI_IT_ERR); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================


#else
//==================================================================================================

#endif
}

void SPIPeriph::intMasterTransfer(uint16_t size,
                                  GenBuffer<uint8_t> *TxBuff, GenBuffer<uint8_t> *RxBuff,
                                  volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication.
 * This version of the SPI Form update functions, is expected to be only used for "Interrupt"
 * based communication, and therefore uses the "GenBuffer" handle input.
 *   This is an OVERLOADED function, used to populate the SPI Request Form, with the Chip select
 *   being managed by the hardware.
 *************************************************************************************************/
    SPIPeriph::Form RequestForm = this->GenericForm(HardwareCS(), size, fltReturn, cmpFlag);
    // Build the generic parts of the SPI Request Form

    this->FormWGenBuffer(&RequestForm, TxBuff, RxBuff);
    // Populate with specific entries for the data type provided as input

    this->FormQueue.InputWrite(RequestForm);
    // Add to queue

    // Trigger interrupt(s)
    this->SPIInterruptStart();
}

void SPIPeriph::intMasterTransfer(GPIO *CS, uint16_t size,
                                  GenBuffer<uint8_t> *TxBuff, GenBuffer<uint8_t> *RxBuff,
                                  volatile DevFlt *fltReturn, volatile uint8_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication.
 * This version of the SPI Form update functions, is expected to be only used for "Interrupt"
 * based communication, and therefore uses the "GenBuffer" handle input.
 *   Second version of the OVERLOADED function.
 *   This version populates the form, but states that the Chip select is managed by the software.
 *************************************************************************************************/
    SPIPeriph::Form RequestForm = this->GenericForm(SoftwareGPIO(CS), size, fltReturn, cmpFlag);

    this->FormWGenBuffer(&RequestForm, TxBuff, RxBuff);
    // Populate with specific entries for the data type provided as input

    this->FormQueue.InputWrite(RequestForm);
    // Add to queue

    // Trigger interrupt(s)
    this->SPIInterruptStart();
}

void SPIPeriph::SPIInterruptStart(void) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication if there is something in the
 * queue, and the bus is free.
 *************************************************************************************************/
    if ( (this->CommState == Free) && (this->FormQueue.State() != GenBuffer_Empty) ) {
        // If the I2C bus is free, and there is I2C request forms in the queue
        this->FormQueue.OutputRead( &(this->curForm) );    // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(this->curForm.Flt) != SPIPeriph::DevFlt::None  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( this->FormQueue.State() == GenBuffer_Empty ) { // If buffer is empty, break out
                this->Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            this->FormQueue.OutputRead( &(this->curForm) );     // Capture form request
        }

        this->CommState = Communicating;        // Lock SPI bus

        this->Enable();

        this->curCount  = this->curForm.size;

        this->ChipSelectHandle(this->curForm.devLoc, Select);
            // Select the specified device location as per SPI Request Form

        this->configReceiveIT(ITEnable);            // Then enable Receive buffer full interrupt
        this->configTransmtIT(ITEnable);            // Then enable Transmit Empty buffer interrupt
    }
    else if ( (this->CommState == Free) && (this->FormQueue.State() == GenBuffer_Empty) ) {
        this->Disable();
    }
}

void SPIPeriph::intReqFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form, to indicate how much data has been completed.
 * Will then de-select the active SPI device.
 * Indicate that the SPI Device is now free for any new communication
 * Disables the Receive/Transmit Interrupts
 *************************************************************************************************/
    *(this->curForm.Cmplt)  += (this->curForm.size - this->curCount);
    // Indicate how many data points have been transfered (curCount should be 0)

    this->ChipSelectHandle(this->curForm.devLoc, Deselect);
    // Deselect the current device, as communication is now complete

    // Indicate that SPI bus is now free, and disable any interrupts
    this->CommState = Free;
    this->configReceiveIT(ITDisable);       // Disable Receive buffer full interrupt
    this->configTransmtIT(ITDisable);       // Disable Transmit empty buffer interrupt
}

void SPIPeriph::IRQHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the SPI events within the I2C class.
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
 *      Transmit Buffer Empty
 *          - If there is new data to be transmitted from source data location as per Request
 *            Form, then this is to be added to the SPI hardware queue.
 *            If this the data to be added to the hardware queue is the last data point to
 *            transmit, then disable the interrupt.
 *            >> NOTE <<
 *              For the STM32L, so as to support single 8bit transmission, this interrupt is
 *              disabled everytime data is added.
 *
 *      Receive Buffer full (not empty)
 *          - Data is to be added to the source location requested as per Request Form. If this is
 *            the last data point. Then the current target SPI device will be "Deselected".
 *            The complete flag will be updated, and if there is still Request Forms to be worked
 *            on, then the next will be selected.
 *
 *      Bus errors:
 *      Overrun
 *          - Set the Fault flag (as per Request Form) to indicate that an Overrun has occurred
 *
 *      Mode Fault
 *          - Set the Fault flag (as per Request Form) to indicate that an Mode fault has
 *            occurred.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    if ( (this->TransmitEmptyChk() & this->TransmitEmptyITChk()) == 0x01) { // If Transmit Buffer
                                                                            // Empty triggered
        this->DRWrite (  this->GetFormWriteData( &(this->curForm) )  );
        // Retrieve next data point from the Request SPI Form, and put onto hardware queue

#if   defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
        // STM32F utilises a TXFIFO, which will only be filled (and remove the interrupt) if
        // 3x8bits are populated within the buffer.
        // This feature of the hardware is not really used within the class, so interrupt is to be
        // disabled once single byte has been put into FIFO.
        // Will be enabled once a read of the hardware has occurred.
        this->configTransmtIT(ITDisable);
#else
//=================================================================================================
        if (this->curCount <= 1)
            this->configTransmtIT(ITDisable);
#endif
    }

    if ( (this->ReceiveToReadChk() & this->ReceiveToReadITChk()) == 0x01) { // If Receive Buffer
                                                                            // full triggered
        this->PutFormReadData( &(this->curForm) , this->DRRead() );
        // Put next data point into the area requested from the SPI Form
        this->curCount--;       // Decrement the class global current count

        if (this->curCount == 0) {
            this->intReqFormCmplt();                // Complete the current request form
                                                    // (no faults)
            this->SPIInterruptStart();              // Check if any new requests
                                                    // remain
        } else {                                    // Only when the count is none zero
            this->configTransmtIT(ITEnable);        // Re-enable the Transmit empty interrupt
        }
    }

    if (this->BusErrorITChk()  == 0x01) {           // If Bus Error interrupt is enabled, then
                                                    // check each of the faults that can cause
                                                    // bus errors
        if (this->BusOverRunChk() == 0x01) {        // If a Bus Over run fault has been detected
            this->ClearBusOvrRun();                 // Clear the failure
            *(this->curForm.Flt)    = DevFlt::Overrun;      // Indicate a data overrun fault

            // Flush the contents of the write buffer
            this->FlushFormWritedData( &(this->curForm) , this->curCount );

            this->intReqFormCmplt();                // Complete the current request form
            this->SPIInterruptStart();              // Check if any new requests
                                                    // remain
        }

        if (this->BusModeFltChk() == 0x01) {        // If a Bus Mode fault has been detected
            this->ClearBusModeFlt();                // Clear the fault (results in the SPI
                                                    // peripheral being disabled
            *(this->curForm.Flt)    = DevFlt::ModeFault;    // Indicate a mode fault has occurred

            // Flush the contents of the write buffer
            this->FlushFormWritedData( &(this->curForm) , this->curCount );

            this->intReqFormCmplt();                // Complete the current request form
            this->SPIInterruptStart();              // Check if any new requests
                                                    // remain
        }
    }

}

SPIPeriph::~SPIPeriph()
{
    // TODO Auto-generated destructor stub
}

