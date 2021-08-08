/**************************************************************************************************
 * @file        AD741x.cpp
 * @author      Thomas
 * @brief       Source file for the AD741x series of temperature sensors
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_AD741x_HD               // Header for AD741x Device

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class
#include FilInd_I2CPe__HD               // Include class for I2C Peripheral

//=================================================================================================

void AD741x::popGenParam(DevPart DeviceNum, AddrBit ASPin) {
/**************************************************************************************************
 * Generate default parameters for the AD741x class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    _address_pin_ = ASPin;              // Pass Address Pin into class parameters
    _part_number_ = DeviceNum;          // Pass Part number into class parameters
    _mode_        = PwrState::kFull_Power;  // Initial state of device is "Full Power"
    _filter_mode_ = FiltState::kEnabled;    // Initial state of device is "Filter Enabled"

    _address_pointer_    = 0xFF;        // Initialise the Address pointer to 0xFF
                                        // This will be corrected on first transfer to device

    _i2c_address = 0x0000;              // Populate I2C address, with 0. Will be updated
                                        // correctly, by next function call
    getI2CAddress();    // Determine the I2C address from provided parameters

    flt           = DevFlt::kInitialised;   // Set fault to initialised

    i2c_wrte_flt      = I2CPeriph::DevFlt::kNone; // I2C write fault status set to "None"
    wrte_cmp_flg      = 0x00;                     // Initialise the communication complete flag
    wrte_cmp_target   = 0x00;                     // Initialise the target communication count

    i2c_read_flt      = I2CPeriph::DevFlt::kNone; // I2C read fault status set to "None"
    read_cmp_flag     = 0x00;                     // Initialise the communication complete flag
    read_cmp_target   = 0x00;                     // Initialise the target communication count

    temp          = -999;               // Default to "-999"
    temp_reg      = 0;                  // Default to "0"
}

AD741x::AD741x(DevPart DeviceNum, AddrBit ASPin, Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * "Lite" class constructor, which requires the GenBuffer pointers for "Address", "read" and
 * "write".
 * Which will then be linked to the class parameters.
 *************************************************************************************************/
    popGenParam(DeviceNum, ASPin);    // Populate generic parameters

    _address_buff_.create(FormArray, FormSize);   // Create GenBuffer for internal Address
                                                        // Pointer queue
    reInitialise();       // Ensure that the internal mechanics of the class have been reset
}

void AD741x::getI2CAddress(void) {
/**************************************************************************************************
 * Function will bring in the PartNumber and _address_pin_ provided to the class, and determine what
 * the I2C address is.
 * This is then put into the class "_i2c_address".
 *************************************************************************************************/
    if      (_part_number_ == DevPart::kAD7414_0) {     // If Device is AD7414-0
        if       (_address_pin_ == AddrBit::kFloat)     // Address pin is floating
            _i2c_address    = AD741x_AD7414_0_Float;    // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kGnd)       // Address pin is grounded
            _i2c_address    = AD741x_AD7414_0_GND;      // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kVdd)       // Address pin is connected to VDD
            _i2c_address    = AD741x_AD7414_0_VDD;      // Setup the I2C Address
    }
    else if (_part_number_ == DevPart::kAD7414_1) {     // If Device is AD7414-1
        if       (_address_pin_ == AddrBit::kFloat)     // Address pin is floating
            _i2c_address    = AD741x_AD7414_1_Float;    // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kGnd)       // Address pin is grounded
            _i2c_address    = AD741x_AD7414_1_GND;      // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kVdd)       // Address pin is connected to VDD
            _i2c_address    = AD741x_AD7414_1_VDD;      // Setup the I2C Address
    }
    else if (_part_number_ == DevPart::kAD7414_2)       // If Device is AD7414-2
        _i2c_address    = AD741x_AD7414_2;              // Setup the I2C Address

    else if (_part_number_ == DevPart::kAD7414_3)       // If Device is AD7414-3
        _i2c_address    = AD741x_AD7414_3;              // Setup the I2C Address

    else if (_part_number_ == DevPart::kAD7415_0) {     // If Device is AD7415-0
        if       (_address_pin_ == AddrBit::kFloat)     // Address pin is floating
            _i2c_address    = AD741x_AD7415_0_Float;    // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kGnd)       // Address pin is grounded
            _i2c_address    = AD741x_AD7415_0_GND;      // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kVdd)       // Address pin is connected to VDD
            _i2c_address    = AD741x_AD7415_0_VDD;      // Setup the I2C Address
    }
    else if (_part_number_ == DevPart::kAD7415_1) {     // If Device is AD7415-1
        if       (_address_pin_ == AddrBit::kFloat)     // Address pin is floating
            _i2c_address    = AD741x_AD7415_1_Float;    // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kGnd)       // Address pin is grounded
            _i2c_address    = AD741x_AD7415_1_GND;      // Setup the I2C Address

        else if  (_address_pin_ == AddrBit::kVdd)       // Address pin is connected to VDD
            _i2c_address    = AD741x_AD7415_1_VDD;      // Setup the I2C Address
      }
}

AD741x::Form AD741x::addressForm(uint8_t newAdd, AD741x::Form::Dir Direction) {
/**************************************************************************************************
 * As class uses a Form structure, this function retrieves the input parameters and will then
 * output a Form structure specific for the AD741x.
 *************************************************************************************************/
    Form new_form = { 0 };

    new_form.AddrPoint  = newAdd;
    new_form.ReadWrite  = Direction;

    return(new_form);
}

uint8_t AD741x::updateAddressPointer(uint8_t *buff, uint8_t newval) {
/**************************************************************************************************
 * Function will put the new Address Pointer request into the input Buffer.
 * Buffer needs to be added, so as to allow for use of other buffers other than the internal
 * write buffer.
 *  Will also up in the new requested value into the "Address Pointer Buffer", so as to ensure
 *  that on time of read back, the class INTERNAL ADDRESS POINTER copy aligns up.
 *
 * Returns the number of bytes that need to be written (which will always be 1)
 *************************************************************************************************/
    _address_buff_.inputWrite(addressForm(newval, Form::kWrite));

    *(buff) = newval;           // Update the input buffer with new entry

    return (1);         // Return number of bytes to be written to I2C
}

uint8_t AD741x::updateConfigReg(uint8_t *buff, PwrState Mode, FiltState Filt, OneShot Conv) {
/**************************************************************************************************
 * Function will put the new Configuration write request into the input "buffer".
 *
 * Returns the number of bytes that need to be written (which will always be 2)
 *************************************************************************************************/
    uint8_t new_reg = 0x00;             // Temporary variable to store the new contents of the
                                        // Configuration Register

    updateAddressPointer(buff, AD741x_ConfigReg);   // Change pointer to "Configuration Reg"

    if (Mode == PwrState::kStand_By) {  // If request is to put device into "StandBy"
        new_reg |= AD741x_PowerDown;    // Set "PowerDown" bit
    }

    if (Filt == FiltState::kEnabled) {  // If request is to enable filter
        new_reg |= AD741x_Filter;       // Set "Filter Enable" bit
    }

    if (Conv == OneShot::kTrigConv) {   // If request is to trigger a conversion
        new_reg |= AD741x_OneShot;      // Set the "OneShot" bit
    }

    buff    += sizeof(uint8_t); // Update pointer
    *(buff) = new_reg;          // Put new Configuration contents into queue

    return (2);         // Return number of bytes to be written to I2C
}

uint8_t AD741x::lastAddresPointRqst(void) {
/**************************************************************************************************
 * Retrieve the last requested update of the Address Pointer.
 *  This is to be used so as to sequence data requests.
 *************************************************************************************************/
    uint16_t last_entry = 0;                    // Variable to retain pointer

    if (_address_buff_.input_pointer == 0)      // If last entry, then previous entry is
        last_entry = (uint16_t) ( _address_buff_.length - 1 );
            // update variable to bottom of Buffer array

    else
        last_entry = (uint16_t) ( (_address_buff_.input_pointer - 1) % _address_buff_.length );

    return(_address_buff_.pa[last_entry].AddrPoint);    // Return the last Address request
}

void AD741x::decodeConfig(uint8_t data) {
/**************************************************************************************************
 * Function will read in the Register state, and then break down what the register states, and
 * bring into class.
 *************************************************************************************************/
    if ( (data & AD741x_PowerDown ) == 0) {     // If power bit is not set
        _mode_  = PwrState::kFull_Power;        // Capture that device is in Full Power, Mode 1
    } else {
        _mode_  = PwrState::kStand_By;          // Capture that device is in Stand By, Mode 2
    }

    if ( (data & AD741x_Filter) == 0 ) {        // If filter bit is disabled
        _filter_mode_   = FiltState::kDisabled; // Capture that filter is disabled
    } else {
        _filter_mode_   = FiltState::kEnabled;  // Capture that filter is enabled
    }
}

void AD741x::decodeTempReg(uint8_t *pData) {
/**************************************************************************************************
 * Function will calculate the temperature reading from the selected device.
 * Expects the first entry to be the MSB, the second to then be LSB (in the format read via I2C).
 *************************************************************************************************/
    uint16_t raw_data = 0;

    raw_data = (uint16_t) ( (pData[0] << 8) | pData[1] );
        // Pack together the 2 Temperature Registers

    raw_data = ((raw_data & AD741x_TempLowerMask) >> AD741x_TempShift);
                // Capture only the Temperature values, and shift down

    temp_reg = (int16_t) ( (raw_data & ~AD741x_SignBit) );
            // Only capture the data values (i.e. ignore sign bit)

    if ((raw_data & AD741x_SignBit) == AD741x_SignBit)  // If sign bit is set, then convert to
        temp_reg -= 512;                                // two's complement equivalent

    temp = ((float)temp_reg) / 4;               // Take value and divide by 4, to get degC
}

AD741x::DevFlt AD741x::deconstructData(uint8_t *readData, uint16_t size) {
/**************************************************************************************************
 * Function will go through the entries within "Read Buffer", and decode them.
 *  First will check to see if the "Address Pointer" was to be updated; entry will be queued
 *  within "Address Pointer Buffer"
 *
 * Then depending upon what the state of the "Address Pointer" is, will determine how the input
 * data is to be understood.
 * If the size of data to be read is higher than the size expected for the source decoder, a fault
 * is triggered
 *************************************************************************************************/
    Form temp_form = { 0 };                 // Temporary form to store current state of
                                            // communication

    while (size != 0) {
        if (_address_buff_.state() != kGenBuffer_Empty) {   // If Address Pointer needs to be
                                                            // updated
            _address_buff_.outputRead(&temp_form);          // Retrieve form for Address Pointer

            _address_pointer_ = temp_form.AddrPoint;       // Copy Address contents to current
                                                        // Address
            if (temp_form.ReadWrite == Form::kWrite) { continue; }
                // If the form states that the communication was a "Write", then no data to
                // read back. So ignore this request.
        }
        else {  // If no entry in queue (but function still expecting to decode data), then
                // indicate Synchronised Error
            return (flt = DevFlt::kSynchronize_Error);
        }

        if      (_address_pointer_ == AD741x_ConfigReg)      {  // If Address is the "Config
                                                                // Register"
            uint8_t tmp = 0;                    // Generate variable to store register state
            tmp = *readData;                    // Put data into variable
            readData    += sizeof(uint8_t);     // Increment array pointer

            decodeConfig(tmp);                  // Decode the Configuration register
            size -= 1;
        }
        else if (_address_pointer_ == AD741x_TemperatureReg) {      // If Address if the
                                                                    // "Temperature Register"
            if (size < 2) { return(flt = DevFlt::kSize_Request); }  // If size is wrong, set
                                                                    // FAULT

            decodeTempReg(readData);                // Convert read data into actual values
            readData    += 2 * (sizeof(uint8_t));   // Increment array pointer (by 2 bytes)
            size -= 2;
        }
        else
            return (flt = DevFlt::kUnrecognised_Address);
    }

    return (flt = DevFlt::kNone);      // Return no fault
}

AD741x::DevFlt AD741x::poleAvailability(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will check to see if the device is available via the I2C link.
 *************************************************************************************************/
    uint8_t i = 0;              // Variable used to loop through the number of available checks

    while (i != 10) {           // Only attempt to check device availability 10 times
        if (hal_I2C->poleDeviceRdy(_i2c_address) != I2CPeriph::DevFlt::kNone) {
            // Check to see if device is available. If device is not available then
            i++;                            // Go for another attempt
        }
        else {                              // If "No Fault" is set
            return(flt = DevFlt::kNone);    // Exit Function, and indicate no fault
        }
    }

    // If all 10 attempts have failed, then indicate a fault with device
    return(flt = DevFlt::kFault);           // Return fault state
}

AD741x::DevFlt AD741x::poleConfigRead(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Configuration Register, and then copy contents into Class.
 *************************************************************************************************/
    uint8_t rData[1] = { 0 };           // Array to contain the Temperature values
    uint8_t packet_size = 0;            // Variable to store the number of bytes to read/write

    if ( (flt == DevFlt::kInitialised) || (_address_pointer_ != AD741x_ConfigReg) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Configuration register.
        // Then request update to the Address pointer
        packet_size = updateAddressPointer(&rData[0], AD741x_ConfigReg);
            // Request Address Pointer update

        // Then transmit the updated request via I2C
        if ( hal_I2C->poleMasterTransmit(
                _i2c_address, &rData[0], packet_size) != I2CPeriph::DevFlt::kNone )
            return (flt = DevFlt::kFault);
    }
    // Ensure the Read of this register is captured.
    _address_buff_.inputWrite(
            addressForm(AD741x_ConfigReg, Form::kRead)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    // Then commence a read of the Temperature Registers
    if ( hal_I2C->poleMasterReceive(_i2c_address, &rData[0], 1) != I2CPeriph::DevFlt::kNone )
        return (flt = DevFlt::kFault);

    return (deconstructData(&rData[0], 1));       // Decode data, and return any faults
                                                        // generated
}

AD741x::DevFlt AD741x::poleConfigWrite(I2CPeriph *hal_I2C,
                        PwrState Mode, FiltState Filt, OneShot Conv) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Configuration Register, and then copy contents into Class.
 *************************************************************************************************/
    uint8_t rData[2] = { 0 };           // Array to contain the Temperature values
    uint8_t packet_size = 0;            // Variable to store the number of bytes to read/write

    packet_size = updateConfigReg(&rData[0], Mode, Filt, Conv);
        // Request a write of the Configuration register

    // Then transmit the updated request via I2C
    if ( hal_I2C->poleMasterTransmit(
            _i2c_address, &rData[0], packet_size) != I2CPeriph::DevFlt::kNone )
        return (flt = DevFlt::kFault);

    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

AD741x::DevFlt AD741x::poleTempRead(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Temperature Register, and then calculate the actual reading.
 *************************************************************************************************/
    uint8_t rData[2] = { 0 };           // Array to contain the Temperature values
    uint8_t packet_size = 0;            // Variable to store the number of bytes to read/write

    if ( (flt == DevFlt::kInitialised) || (_address_pointer_ != AD741x_TemperatureReg) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Temperature register.
        // Then request update to the Address pointer
        packet_size = updateAddressPointer(&rData[0], AD741x_TemperatureReg);
            // Request Address Pointer update

        // Then transmit the updated request via I2C
        if ( hal_I2C->poleMasterTransmit(
                _i2c_address, &rData[0], packet_size) != I2CPeriph::DevFlt::kNone )
            return (flt = DevFlt::kFault);
    }
    // Ensure the Read of this register is captured.
    _address_buff_.inputWrite(
            addressForm(AD741x_TemperatureReg, Form::kRead)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    // Then commence a read of the Temperature Registers
    if ( hal_I2C->poleMasterReceive(_i2c_address, &rData[0], 2) != I2CPeriph::DevFlt::kNone )
        return (flt = DevFlt::kFault);

    return (deconstructData(&rData[0], 2));       // Decode data, and return any faults
                                                        // generated
}

void AD741x::reInitialise(void) {
/**************************************************************************************************
 * This function is to be called when the internal mechanics of the class need to be reset. This
 * is likely to occur in fault situations, or at initialisation.
 *************************************************************************************************/
    _address_buff_.qFlush();                // Clear the Address Pointer queue
    _address_pointer_    = 0xFF;            // Initialise the Address pointer to 0xFF
                                            // This will be corrected on first transfer to device
    flt           = DevFlt::kInitialised;   // Set fault to initialised
}

void AD741x::intConfigRead(I2CPeriph *hal_I2C, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for the contents of the Configuration Register
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;         // Variable to store the size of request

    if ( (flt == DevFlt::kInitialised) ||
         (lastAddresPointRqst() != AD741x_ConfigReg) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Temperature Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], AD741x_ConfigReg);

        hal_I2C->intMasterReq(_i2c_address,
                              temp_size,
                              &wBuff[wrte_cmp_target],
                              I2CPeriph::CommMode::kAutoEnd, I2CPeriph::Request::kStart_Write,
                              &(i2c_wrte_flt), &(wrte_cmp_flg));

        wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
    }
    // Ensure the Read of this register is captured.
    _address_buff_.inputWrite(
            addressForm(AD741x_ConfigReg, Form::kRead)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    hal_I2C->intMasterReq(_i2c_address,
                          1,
                          &rBuff[read_cmp_target],
                          I2CPeriph::CommMode::kAutoEnd, I2CPeriph::Request::kStart_Read,
                          &(i2c_read_flt), &(read_cmp_flag));

    read_cmp_target   += 1;     // Put expected size of read back into read complete target
}

void AD741x::intConfigWrite(I2CPeriph *hal_I2C,
                            PwrState Mode, FiltState Filt, OneShot Conv, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for updating the contents of the Configuration Register
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;          // Variable to store the size of request

    temp_size = updateConfigReg(&wBuff[wrte_cmp_target], Mode, Filt, Conv);

    hal_I2C->intMasterReq(_i2c_address,
                          temp_size,
                          &wBuff[wrte_cmp_target],
                          I2CPeriph::CommMode::kAutoEnd, I2CPeriph::Request::kStart_Write,
                          &(i2c_wrte_flt), &(wrte_cmp_flg));

    wrte_cmp_target   += temp_size; // Copy expected size to write complete target
}

void AD741x::intTempRead(I2CPeriph *hal_I2C, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request of temperature read of the AD741x device.
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;          // Variable to store the size of request

    if ( (flt == DevFlt::kInitialised) ||
         (lastAddresPointRqst() != AD741x_TemperatureReg) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Temperature Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], AD741x_TemperatureReg);

        hal_I2C->intMasterReq(_i2c_address,
                              temp_size,
                              &wBuff[wrte_cmp_target],
                              I2CPeriph::CommMode::kAutoEnd, I2CPeriph::Request::kStart_Write,
                              &(i2c_wrte_flt), &(wrte_cmp_flg));

        wrte_cmp_target   += temp_size; // Copy expected size to write complete target
    }
    // Ensure the Read of this register is captured.
    _address_buff_.inputWrite(
            addressForm(AD741x_TemperatureReg, Form::kRead)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    hal_I2C->intMasterReq(_i2c_address,
                          2,
                          &rBuff[read_cmp_target],
                          I2CPeriph::CommMode::kAutoEnd, I2CPeriph::Request::kStart_Read,
                          &(i2c_read_flt), &(read_cmp_flag));

    read_cmp_target   += 2;     // Put expected size of read back into read complete target
}

void AD741x::intCheckCommStatus(uint8_t *rBuff, uint16_t size) {
/**************************************************************************************************
 * Function needs to be called periodically, as this is used to check status of the interrupt
 * based communication, and then deconstruct the data if there is any available.
 *************************************************************************************************/
    deconstructData(rBuff, size);         // Deconstruct data
}

AD741x::~AD741x()
{
    // TODO Auto-generated destructor stub
}

