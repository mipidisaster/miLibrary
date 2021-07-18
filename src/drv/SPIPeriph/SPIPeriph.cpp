/**************************************************************************************************
 * @file        SPIPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic SPIPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/

#include <FileIndex.h>
#include FilInd_SPIPe__HD

void SPIPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the SPIPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    Flt           = DevFlt::kInitialised;     // Initialise the fault to "initialised"
    CommState     = CommLock::kFree;          // Indicate bus is free

    _cur_count_   = 0;                // Initialise the current packet size count

    _cur_form_    = { 0 };            // Initialise the form to a blank entry
}

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
SPIPeriph::SPIPeriph(SPI_HandleTypeDef *SPIHandle, Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for the STM32 device
 * Receives the address of the SPI Handle of device - generated from cubeMX
 * Will then determine which mode has been selected, based upon the states of the registers
 *************************************************************************************************/
    popGenParam();                    // Populate generic class parameters

    _spi_handle_        = SPIHandle;  // copy handle across into class

    _form_queue_.create(FormArray, FormSize);

    // From handle can determine what the MODE of the SPI can be configured too:
    if (_spi_handle_->Instance->CR1 & SPI_POLARITY_HIGH) {  // If Clock Idles HIGH
        if (_spi_handle_->Instance->CR1 & SPI_PHASE_2EDGE)  // If data is captured on second edge
            _mode_ = SPIMode::kMode3;                       // MODE is 3
        else                                                // Otherwise captured on first edge
            _mode_ = SPIMode::kMode2;                       // MODE is 2
    } else {                                                // If Clock Idles LOW
        if (_spi_handle_->Instance->CR1 & SPI_PHASE_2EDGE)  // If data is captured on second edge
            _mode_ = SPIMode::kMode1;                       // MODE is 1
        else                                                // Otherwise captured on first edge
            _mode_ = SPIMode::kMode0;                       // MODE is 0
    }

    enable();                     // Enable the SPI device
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
SPIPeriph::SPIPeriph(int channel, int speed, _SPIMode Mode) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for RaspberryPi
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    popGenParam();                // Populate generic class parameters

    _mode_        = Mode;         // Copy across the selected Mode
    _spi_handle_  = channel;      //
    int tempMode = 0;

    if      (_mode_ == SPIMode::kMode1) // If Mode 1 is selected then
        tempMode = 1;                   // Store "1"
    else if (_mode_ == SPIMode::kMode2) // If Mode 2 is selected then
        tempMode = 2;                   // Store "2"
    else if (_mode_ == SPIMode::kMode3) // If Mode 3 is selected then
        tempMode = 3;                   // Store "3"
    else                                // If any other Mode is selected then
       tempMode = 0;                    // Default to "0"

    wiringPiSPISetupMode(_spi_channel_, speed, tempMode);
        // Enable SPI interface for selected SPI channel, speed and mode
}
#else
//=================================================================================================
SPIPeriph::SPIPeriph() {
}
#endif

void SPIPeriph::enable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    __HAL_SPI_ENABLE(_spi_handle_);     // Enable the SPI interface

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_SPI_ENABLE(_spi_handle_);     // Enable the SPI interface

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    // Done in initial call.

#else
//=================================================================================================

#endif
}

void SPIPeriph::disable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    __HAL_SPI_DISABLE(_spi_handle_);    // Disable the SPI interface

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_SPI_DISABLE(_spi_handle_);    // Disable the SPI interface

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    // Done in initial call.

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::readDR(void) {
/**************************************************************************************************
 * Read from the SPI hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    return ((uint8_t) _spi_handle_->Instance->DR);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// STM32L4 uses a RXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)
    return( (uint8_t) _spi_handle_->Instance->DR );

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return 0;

#else
//=================================================================================================

#endif
}

void SPIPeriph::writeDR(uint8_t data) {
/**************************************************************************************************
 * Write to the SPI hardware
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    _spi_handle_->Instance->DR = data;

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// STM32L4 uses a TXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)
// To ensure only 8bits is transmitted, need to ensure that we cast the Data Register (DR) to
// unsigned 8bits, hence the initial casing
    *(uint8_t *)&_spi_handle_->Instance->DR = data;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
#endif
}

uint8_t SPIPeriph::transmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::receiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::busBusyChk(void) {
/**************************************************************************************************
 * Check to see if the SPI bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_BSY) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_BSY) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::busOverRunChk(void) {
/**************************************************************************************************
 * Check to see if a Bus Over run has occurred (if over run, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_OVR) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_OVR) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions
    return (0);

#else
//=================================================================================================

#endif
}

void SPIPeriph::clearBusOvrRun(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Overrun fault state
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// To clear the Bus Over run status form STM32F, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(_spi_handle_);      // Utilise the existing MACRO

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// To clear the Bus Over run status form STM32L, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(_spi_handle_);      // Utilise the existing MACRO


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::busModeFltChk(void) {
/**************************************************************************************************
 * Check to see if a Bus mode fault has occurred (if mode fault, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_MODF) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_MODF) != 0x00 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

void SPIPeriph::clearBusModeFlt(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Mode fault state
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(_spi_handle_);     // Utilise the existing MACRO

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(_spi_handle_);     // Utilise the existing MACRO


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::transmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_TXE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_TXE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::receiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

uint8_t SPIPeriph::busErrorITChk(void) {
/**************************************************************************************************
 * Check to see whether the Bus Error interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_ERR) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_ERR) != 0 )
        return (1);
    else
        return (0);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library. Function will not be
// called by upper level functions

#else
//=================================================================================================

#endif
}

SPIPeriph::CSHandle SPIPeriph::hardwareCS(void) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via the Hardware Chip Select (no software interaction required)
 *************************************************************************************************/
    CSHandle temp_struct = { 0 };                       // Initialise structure

    temp_struct.Type  = CSHandle::CSType::kHardware_Managed;    // Indicate that SPI device is
                                                                // hardware managed
    return (temp_struct);                               // Return build structure
}

SPIPeriph::CSHandle SPIPeriph::softwareGPIO(GPIO *CS) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via software selection of input GPIO pin.
 *************************************************************************************************/
    CSHandle temp_struct = { 0 };                       // Initialise structure

    temp_struct.GPIO_CS  = CS;                          // Link input Chip Select to struct
    temp_struct.Type  = CSHandle::CSType::kSoftware_GPIO;   // Indicate that SPI device is software
                                                            // managed
    return (temp_struct);                               // Return build structure
}

void SPIPeriph::chipSelectHandle(CSHandle selection, CSSelection Mode) {
/**************************************************************************************************
 * With the provided CSHandle struct, and the desired Mode (Select/Deselect), function will then
 * do the necessary work to enable the external SPI device for communication.
 *************************************************************************************************/
    if (selection.Type == CSHandle::CSType::kSoftware_GPIO) {   // If the CSHandle indicates
                                                                // Software management, then...
        if (Mode == CSSelection::kSelect)                   // If Mode is "Select"
            selection.GPIO_CS->setValue(GPIO::kLow);        // Pull the GPIO "LOW"

        else                                                // If Mode is "DeSelect"
            selection.GPIO_CS->setValue(GPIO::kHigh);       // Pull the GPIO "HIGH"
    }
    // Hardware Managed doesn't require any software interaction. Therefore doesn't do anything
    // is selected.
}

SPIPeriph::Form SPIPeriph::genericForm(CSHandle devLoc, uint16_t size,
                                       volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Generate a SPIForm request, based upon the generic information provided as input.
 *************************************************************************************************/
    Form request_form = { 0 };                  // Generate the "Form" variable to provide as 
                                                // output

    request_form.devLoc          = devLoc;      // Populate form with input data
    request_form.size            = size;        //

    // Indications used for source functionality to get status of requested communication
    request_form.Flt             = fltReturn;   // Populate return fault flag
    request_form.Cmplt           = cmpFlag;     // Populate complete communication indication

    return (request_form);
}

void SPIPeriph::formW8bitArray(Form *RequestForm, uint8_t *TxData, uint8_t *RxData) {
/**************************************************************************************************
 * Link input 8bit array pointer(s) to the provided SPI Request Form.
 *************************************************************************************************/
    RequestForm->TxBuff     = TxData;           // Pass Transmit Buffer to SPIForm

    RequestForm->RxBuff     = RxData;           // Pass Transmit Buffer to SPIForm
}

uint8_t SPIPeriph::getFormWriteData(Form *RequestForm) {
/**************************************************************************************************
 * Retrieve the next data point to write to external device from the selected SPI Request form
 *************************************************************************************************/
    uint8_t temp_val = 0;       // Temporary variable to store data value

    temp_val = *(RequestForm->TxBuff);          // Retrieve data from array
    RequestForm->TxBuff     += sizeof(uint8_t); // Increment array pointer

    return (temp_val);          // Return value outside of function
}

void SPIPeriph::putFormReadData(Form *RequestForm, uint8_t readdata) {
/**************************************************************************************************
 * Data read from the SPI external device is copied into the requested source location, as per
 * the SPI Request form
 *************************************************************************************************/
    *(RequestForm->RxBuff)  = readdata;         // Put data into array
    RequestForm->RxBuff     += sizeof(uint8_t); // Increment array pointer
}

SPIPeriph::DevFlt SPIPeriph::poleMasterTransfer(uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the external device; selected by the Chip Select
 * Pin. The Master being this local device.
 *   This is an OVERLOADED function, and forms the bases of the poling transmission for SPI. This
 *   version doesn't pull down any pins within software. Relies upon a hardware managed CS.
 *************************************************************************************************/
    if (wData == __null || rData == __null || size == 0)    // If no data has been requested
        return ( Flt = DevFlt::kData_Size );                // to be set return error

    // Indicate that the bus is not free
    CommState = CommLock::kCommunicating;       // Indicate bus is communicating

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    enable();                       // Ensure that the device has been enabled

    SET_BIT(_spi_handle_->Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);
        // Ensure that the RXNE bit is set when the Receive buffer is 1/4 full (i.e. 8bits is
        // within)

    while (size != 0) {
        // Wait for the transfer buffer to be empty before putting data for transfer, whilst
        // waiting, check for any faults
        while(transmitEmptyChk() == 0) {
            if (busOverRunChk() == 1) {     // Any Bus Over runs errors
                clearBusOvrRun();           // Clear the Bus Overrun fault
                return ( Flt = DevFlt::kOverrun );      // Indicate fault, and exit
            }

            if (busModeFltChk() == 1) {     // Any Bus Mode faults detected
                clearBusModeFlt();          // Clear the Bus Mode fault
                return ( Flt = DevFlt::kMode_Fault );   // Indicate fault, and exit
            }
        };
            // No timeout period has been specified - Can get stuck

        writeDR(*wData);                                // Put data onto buffer for transfer

        // Wait for the transfer to complete, and data to be read back from SLAVE, whilst waiting
        // check for any faults
        while(receiveToReadChk() == 0) {
            if (busOverRunChk() == 1) {     // Any Bus Over runs errors
                clearBusOvrRun();           // Clear the Bus Overrun fault
                return ( Flt = DevFlt::kOverrun );      // Indicate fault, and exit
            }

            if (busModeFltChk() == 1) {     // Any Bus Mode faults detected
                clearBusModeFlt();          // Clear the Bus Mode fault
                return ( Flt = DevFlt::kMode_Fault );   // Indicate fault, and exit
            }
        };

        *rData = readDR();          // Put data read from device back into array
        wData += sizeof(uint8_t);   // Increment pointer for write array by the size of the data
                                    // type.
        rData += sizeof(uint8_t);   // Increment pointer for read array by the size of the data
                                    // type.
        size--;                     // Decrement count
    }

    // Only exit function once transfer is complete -> detected by BUSY flag clearing
    while(busBusyChk() == 1) {};

    disable();                      // Ensure that the device has been disable

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    uint16_t i = 0;

    for (i = 0; i != size; i++)         // Cycle through the data to be written
        rData[i] = wData[i];            // and copy into the read data


    wiringPiSPIDataRW(_spi_channel_, rData, (int)size);
        // Using wiringPiSPI function, transfer data from RaspberryPi to selected device

#else
//=================================================================================================

#endif

    // Indicate that the bus is free
    CommState = CommLock::kFree;        // Indicate bus is free

    return ( Flt = DevFlt::kNone );
}


SPIPeriph::DevFlt SPIPeriph::poleMasterTransfer(GPIO *ChipSelect,
                                        uint8_t *wData, uint8_t *rData, uint16_t size) {
/**************************************************************************************************
 * Second version of the OVERLOADED function.
 * This version utilises the basic version, however prior to calling this, pulls down the input
 * GPIO pin.
 *************************************************************************************************/
    DevFlt return_value = DevFlt::kInitialised;         // Variable to store output of RWTransfer

    chipSelectHandle(softwareGPIO(ChipSelect), CSSelection::kSelect);

    return_value = poleMasterTransfer(wData, rData, size);
        // Use private function to transfer data

    chipSelectHandle(softwareGPIO(ChipSelect), CSSelection::kDeselect);

    return(return_value);                               // Return providing the output of transfer
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
    DevFlt return_value = DevFlt::kInitialised;     // Variable to store output of RWTransfer

    DeMuxCS->updateSelection(CSNum);                // Setup Demultiplexor for SPI Slave
                                                    // Chip Select
    DeMuxCS->enable();                              // Enable the Demultiplexor
            // This expects that at least 1 of the EnableLow pins has been linked to the
            // SPI hardware CS signal (will not has been allocated to the DeMuxCS entry)

    return_value = poleMasterTransfer(wData, rData, size);
        // Use private function to transfer data

    DeMuxCS->disable();                             // Disable the Demultiplexor

    return(return_value);                           // Return providing the output of transfer
}

void SPIPeriph::configTransmtIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_TXE);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_TXE); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================


#else
//=================================================================================================

#endif
}

void SPIPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_RXNE); // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_RXNE);// Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================


#else
//=================================================================================================

#endif
}

void SPIPeriph::configBusErroIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus Error interrupt
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_ERR);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_ERR); // Then disable interrupt
    }


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================


#else
//=================================================================================================

#endif
}

void SPIPeriph::intMasterTransfer(uint16_t size, uint8_t *TxBuff, uint8_t *RxBuff,
                                  volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication.
 * Two arrays are provided which will contain the data to transmit, and the location to store
 * read back data.
 *   This is an OVERLOADED function, used to populate the SPI Request Form, with the Chip select
 *   being managed by the hardware.
 *************************************************************************************************/
    Form request_form = genericForm(hardwareCS(), size, fltReturn, cmpFlag);
    // Build the generic parts of the SPI Request Form

    formW8bitArray(&request_form, TxBuff, RxBuff);
    // Populate with specific entries for the data type provided as input

    _form_queue_.inputWrite(request_form);
    // Add to queue

    // Trigger interrupt(s)
    startInterrupt();
}

void SPIPeriph::intMasterTransfer(GPIO *CS, uint16_t size, uint8_t *TxBuff, uint8_t *RxBuff,
                                  volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication.
 * Two arrays are provided which will contain the data to transmit, and the location to store
 * read back data.
 *   Second version of the OVERLOADED function.
 *   This version populates the form, but states that the Chip select is managed by the software.
 *************************************************************************************************/
    Form request_form = genericForm(softwareGPIO(CS), size, fltReturn, cmpFlag);

    formW8bitArray(&request_form, TxBuff, RxBuff);
    // Populate with specific entries for the data type provided as input

    _form_queue_.inputWrite(request_form);
    // Add to queue

    // Trigger interrupt(s)
    startInterrupt();
}

void SPIPeriph::startInterrupt(void) {
/**************************************************************************************************
 * Function will be called to start off a new SPI communication if there is something in the
 * queue, and the bus is free.
 *************************************************************************************************/
    if ( (CommState == CommLock::kFree) && (_form_queue_.state() != kGenBuffer_Empty) ) {
        // If the I2C bus is free, and there is I2C request forms in the queue
        _form_queue_.outputRead( &(_cur_form_) );       // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_form_.Flt) != SPIPeriph::DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_queue_.state() == kGenBuffer_Empty ) {   // If buffer is empty, break out
                disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_queue_.outputRead( &(_cur_form_) );           // Capture form request
        }

        CommState = CommLock::kCommunicating;   // Lock SPI bus

        enable();

        _cur_count_  = _cur_form_.size;

        chipSelectHandle(_cur_form_.devLoc, CSSelection::kSelect);
            // Select the specified device location as per SPI Request Form

        configReceiveIT(InterState::kIT_Enable);    // Then enable Receive buffer full interrupt
        configTransmtIT(InterState::kIT_Enable);    // Then enable Transmit Empty buffer interrupt
    }
    else if ( (CommState == CommLock::kFree) && (_form_queue_.state() == kGenBuffer_Empty) ) {
        disable();
    }
}

void SPIPeriph::intReqFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form, to indicate how much data has been completed.
 * Will then de-select the active SPI device.
 * Indicate that the SPI Device is now free for any new communication
 * Disables the Receive/Transmit Interrupts
 *************************************************************************************************/
    *(_cur_form_.Cmplt)  += (_cur_form_.size - _cur_count_);
    // Indicate how many data points have been transfered (curCount should be 0)

    chipSelectHandle(_cur_form_.devLoc, CSSelection::kDeselect);
    // Deselect the current device, as communication is now complete

    // Indicate that SPI bus is now free, and disable any interrupts
    CommState = CommLock::kFree;
    configReceiveIT(InterState::kIT_Disable);   // Disable Receive buffer full interrupt
    configTransmtIT(InterState::kIT_Disable);   // Disable Transmit empty buffer interrupt
}

void SPIPeriph::handleIRQ(void) {
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
    if ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01) { // If Transmit Buffer Empty
                                                                // triggered
        writeDR (  getFormWriteData( &(_cur_form_) )  );
        // Retrieve next data point from the Request SPI Form, and put onto hardware queue

#if   defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
        // STM32F utilises a TXFIFO, which will only be filled (and remove the interrupt) if
        // 3x8bits are populated within the buffer.
        // This feature of the hardware is not really used within the class, so interrupt is to be
        // disabled once single byte has been put into FIFO.
        // Will be enabled once a read of the hardware has occurred.
        configTransmtIT(InterState::kIT_Disable);
#else
//=================================================================================================
        if (_cur_count_ <= 1)
            configTransmtIT(InterState::kIT_Disable);
#endif
    }

    if ( (receiveToReadChk() & receiveToReadITChk()) == 0x01) { // If Receive Buffer full triggered
        putFormReadData( &(_cur_form_) , readDR() );
        // Put next data point into the area requested from the SPI Form
        _cur_count_--;                  // Decrement the class global current count

        if (_cur_count_ == 0) {
            intReqFormCmplt();          // Complete the current request form (no faults)
            startInterrupt();           // Check if any new requests remain
        } else {                                        // Only when the count is none zero
            configTransmtIT(InterState::kIT_Enable);    // Re-enable the Transmit empty interrupt
        }
    }

    if (busErrorITChk()  == 0x01) {     // If Bus Error interrupt is enabled, then check each of
                                        // the faults that can cause bus errors
        if (busOverRunChk() == 0x01) {  // If a Bus Over run fault has been detected
            clearBusOvrRun();                           // Clear the failure
            *(_cur_form_.Flt)    = DevFlt::kOverrun;    // Indicate a data overrun fault

            intReqFormCmplt();                          // Complete the current request form
            startInterrupt();                           // Check if any new requests remain
        }

        if (busModeFltChk() == 0x01) {  // If a Bus Mode fault has been detected
            clearBusModeFlt();          // Clear the fault (results in the SPI peripheral being
                                        // disabled)
            *(_cur_form_.Flt)    = DevFlt::kMode_Fault; // Indicate a mode fault has occurred

            intReqFormCmplt();          // Complete the current request form
            startInterrupt();           // Check if any new requests remain
        }
    }

}

SPIPeriph::~SPIPeriph()
{
    // TODO Auto-generated destructor stub
}

