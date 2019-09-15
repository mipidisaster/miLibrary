/**************************************************************************************************
 * @file        AD741x.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        15 Sept 2019
 * @brief       Source file for the AD741x series of temperature sensors
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_AD741x_HD

void AD741x::popGenParam(DevPart DeviceNum, AddrBit ASPin) {
/**************************************************************************************************
 * Generate default parameters for the AD741x class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    this->AddressPin    = ASPin;            // Pass Address Pin into class parameters
    this->PartNumber    = DeviceNum;        // Pass Part number into class parameters
    this->Mode          = PwrState::FullPower;      // Initial state of device is "Full Power"
    this->FiltMode      = FiltState::Enabled;       // Initial state of device is "Filter Enabled"

    this->AddressPointer    = 0xFF;         // Initialise the Address pointer to 0xFF
                                            // This will be corrected on first transfer to device

    this->I2CAddress = 0x0000;              // Populate I2C address, with 0. Will be updated
                                            // correctly, by next function call
    this->GetAddress();     // Determine the I2C address from provided parameters

    this->Flt           = DevFlt::Initialised;      // Set fault to initialised

    this->I2CWFlt       = I2CPeriph::DevFlt::None;  // I2C write fault status set to "None"
    this->wtcmpFlag     = 0x00;                     // Initialise the communication complete flag
    this->wtcmpTarget   = 0x00;                     // Initialise the target communication count

    this->I2CRFlt       = I2CPeriph::DevFlt::None;  // I2C read fault status set to "None"
    this->rdcmpFlag     = 0x00;                     // Initialise the communication complete flag
    this->rdcmpTarget   = 0x00;                     // Initialise the target communication count

    this->Temp          = -999;                 // Default to "-999"
    this->TempReg       = 0;                    // Default to "0"
}

AD741x::AD741x(void) {
/**************************************************************************************************
 * Basic construction of AD741x Device
 *************************************************************************************************/
    this->popGenParam(DevPart::AD7414_0, AddrBit::Float);   // Populate generic parameters
}

void AD741x::create(DevPart DeviceNum, AddrBit ASPin, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * "Lite" class constructor, which requires the GenBuffer pointers for "Address", "read" and
 * "write".
 * Which will then be linked to the class parameters.
 *************************************************************************************************/
    this->popGenParam(DeviceNum, ASPin);    // Populate generic parameters

    this->AdBuff.create(FormArray, FormSize);   // Create GenBuffer for internal Address Pointer
                                                // queue
    this->reInitialise();       // Ensure that the internal mechanics of the class have been reset
}

AD741x::AD741x(DevPart DeviceNum, AddrBit ASPin, Form *FormArray, uint32_t FormSize) {
/**************************************************************************************************
 * "Lite" class constructor, which requires the GenBuffer pointers for "Address", "read" and
 * "write".
 * Which will then be linked to the class parameters.
 *************************************************************************************************/
    this->create(DeviceNum, ASPin, FormArray, FormSize);
}

void AD741x::GetAddress(void) {
/**************************************************************************************************
 * Function will bring in the PartNumber and AddressPin provided to the class, and determine what
 * the I2C address is.
 * This is then put into the class "I2CAddress".
 *************************************************************************************************/
    if      (this->PartNumber == DevPart::AD7414_0) {       // If Device is AD7414-0
        if       (this->AddressPin == AddrBit::Float)       // Address pin is floating
            this->I2CAddress    = AD741x_AD7414_0_Float;    // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::GND)         // Address pin is grounded
            this->I2CAddress    = AD741x_AD7414_0_GND;      // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::Vdd)         // Address pin is connected to VDD
            this->I2CAddress    = AD741x_AD7414_0_VDD;      // Setup the I2C Address
    }
    else if (this->PartNumber == DevPart::AD7414_1) {       // If Device is AD7414-1
        if       (this->AddressPin == AddrBit::Float)       // Address pin is floating
            this->I2CAddress    = AD741x_AD7414_1_Float;    // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::GND)         // Address pin is grounded
            this->I2CAddress    = AD741x_AD7414_1_GND;      // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::Vdd)         // Address pin is connected to VDD
            this->I2CAddress    = AD741x_AD7414_1_VDD;      // Setup the I2C Address
    }
    else if (this->PartNumber == DevPart::AD7414_2)         // If Device is AD7414-2
        this->I2CAddress    = AD741x_AD7414_2;              // Setup the I2C Address

    else if (this->PartNumber == DevPart::AD7414_3)         // If Device is AD7414-3
        this->I2CAddress    = AD741x_AD7414_3;              // Setup the I2C Address

    else if (this->PartNumber == DevPart::AD7415_0) {       // If Device is AD7415-0
        if       (this->AddressPin == AddrBit::Float)       // Address pin is floating
            this->I2CAddress    = AD741x_AD7415_0_Float;    // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::GND)         // Address pin is grounded
            this->I2CAddress    = AD741x_AD7415_0_GND;      // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::Vdd)         // Address pin is connected to VDD
            this->I2CAddress    = AD741x_AD7415_0_VDD;      // Setup the I2C Address
    }
    else if (this->PartNumber == DevPart::AD7415_1) {       // If Device is AD7415-1
        if       (this->AddressPin == AddrBit::Float)       // Address pin is floating
            this->I2CAddress    = AD741x_AD7415_1_Float;    // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::GND)         // Address pin is grounded
            this->I2CAddress    = AD741x_AD7415_1_GND;      // Setup the I2C Address

        else if  (this->AddressPin == AddrBit::Vdd)         // Address pin is connected to VDD
            this->I2CAddress    = AD741x_AD7415_1_VDD;      // Setup the I2C Address
      }
}

AD741x::Form AD741x::AddressForm(uint8_t newAdd, AD741x::Form::Dir Direction) {
/**************************************************************************************************
 * As class uses a Form structure, this function retrieves the input parameters and will then
 * output a Form structure specific for the AD741x.
 *************************************************************************************************/
    Form newForm = { 0 };

    newForm.AddrPoint   = newAdd;
    newForm.ReadWrite   = Direction;

    return(newForm);
}

uint8_t AD741x::UpdateAddressPointer(uint8_t *buff, uint8_t newval) {
/**************************************************************************************************
 * Function will put the new Address Pointer request into the input Buffer.
 * Buffer needs to be added, so as to allow for use of other buffers other than the internal
 * write buffer.
 *  Will also up in the new requested value into the "Address Pointer Buffer", so as to ensure
 *  that on time of read back, the class INTERNAL ADDRESS POINTER copy aligns up.
 *
 * Returns the number of bytes that need to be written (which will always be 1)
 *************************************************************************************************/
    this->AdBuff.InputWrite(this->AddressForm(newval, Form::Write));

    *(buff) = newval;           // Update the input buffer with new entry

    return (1);         // Return number of bytes to be written to I2C
}

uint8_t AD741x::UpdateConfigReg(uint8_t *buff, PwrState Mode, FiltState Filt, OneShot Conv) {
/**************************************************************************************************
 * Function will put the new Configuration write request into the input "buffer".
 *
 * Returns the number of bytes that need to be written (which will always be 2)
 *************************************************************************************************/
    uint8_t newreg = 0x00;              // Temporary variable to store the new contents of the
                                        // Configuration Register

    this->UpdateAddressPointer(buff, AD741x_ConfigReg);   // Change pointer to "Configuration Reg"

    if (Mode == PwrState::StandBy) {    // If request is to put device into "StandBy"
        newreg |= AD741x_PowerDown;     // Set "PowerDown" bit
    }

    if (Filt == FiltState::Enabled) {   // If request is to enable filter
        newreg |= AD741x_Filter;        // Set "Filter Enable" bit
    }

    if (Conv == OneShot::TrigConv) {    // If request is to trigger a conversion
        newreg |= AD741x_OneShot;       // Set the "OneShot" bit
    }

    buff    += sizeof(uint8_t); // Update pointer
    *(buff) = newreg;           // Put new Configuration contents into queue

    return (2);         // Return number of bytes to be written to I2C
}

uint8_t AD741x::LastAddresPointRqst(void) {
/**************************************************************************************************
 * Retrieve the last requested update of the Address Pointer.
 *  This is to be used so as to sequence data requests.
 *************************************************************************************************/
    uint32_t lastentry = 0;                 // Variable to retain pointer

    if (this->AdBuff.input_pointer == 0)        // If last entry, then previous entry is
        lastentry = this->AdBuff.length - 1;    // update variable to bottom of Buffer array

    else
        lastentry = (this->AdBuff.input_pointer - 1) % this->AdBuff.length;

    return(this->AdBuff.pa[lastentry].AddrPoint);   // Return the last Address request
}

void AD741x::DecodeConfig(uint8_t data) {
/**************************************************************************************************
 * Function will read in the Register state, and then break down what the register states, and
 * bring into class.
 *************************************************************************************************/
    if ( (data & AD741x_PowerDown ) == 0) {     // If power bit is not set
        this->Mode  = PwrState::FullPower;      // Capture that device is in Full Power, Mode 1
    } else {
        this->Mode  = PwrState::StandBy;        // Capture that device is in Stand By, Mode 2
    }

    if ( (data & AD741x_Filter) == 0 ) {            // If filter bit is disabled
        this->FiltMode = FiltState::Disabled;       // Capture that filter is disabled
    } else {
        this->FiltMode = FiltState::Enabled;        // Capture that filter is enabled
    }
}

void AD741x::DecodeTempReg(uint8_t *pData) {
/**************************************************************************************************
 * Function will calculate the temperature reading from the selected device.
 * Expects the first entry to be the MSB, the second to then be LSB (in the format read via I2C).
 *************************************************************************************************/
    uint16_t rawdata = 0;

    rawdata = (pData[0] << 8) | pData[1];       // Pack together the 2 Temperature Registers
    rawdata = ((rawdata & AD741x_TempLowerMask) >> AD741x_TempShift);
                // Capture only the Temperature values, and shift down

    this->TempReg = (rawdata & ~AD741x_SignBit);    // Only capture the data values (i.e. ignore
                                                    // sign bit)

    if ((rawdata & AD741x_SignBit) == AD741x_SignBit)   // If sign bit is set, then convert to
        this->TempReg -= 512;                           // two's complement equivalent

    this->Temp = ((float)this->TempReg) / 4;            // Take value and divide by 4, to get degC
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
    Form TempForm = { 0 };                  // Temporary form to store current state of
                                            // communication

    while (size != 0) {
        if (this->AdBuff.State() != GenBuffer_Empty) {  // If Address Pointer needs to be updated
            this->AdBuff.OutputRead(&TempForm);         // Retrieve form for Address Pointer

            this->AddressPointer = TempForm.AddrPoint;  // Copy Address contents to current
                                                        // Address
            if (TempForm.ReadWrite == Form::Write) { continue; }
                // If the form states that the communication was a "Write", then no data to
                // read back. So ignore this request.
        }
        else {  // If no entry in queue (but function still expecting to decode data), then
                // indicate Syncrhronized Error
            return (this->Flt = DevFlt::SynchronizeErr);
        }

        if      (this->AddressPointer == AD741x_ConfigReg)      {   // If Address is the "Config
                                                                    // Register"
            uint8_t tmp = 0;                    // Generate variable to store register state
            tmp = *readData;                    // Put data into variable
            readData    += sizeof(uint8_t);     // Increment array pointer

            this->DecodeConfig(tmp);            // Decode the Configuration register
            size -= 1;
        }
        else if (this->AddressPointer == AD741x_TemperatureReg) {   // If Address if the
                                                                    // "Temperature Register"
            if (size < 2) { return(this->Flt = DevFlt::SizeReqst); }    // If size is wrong, set
                                                                        // FAULT

            this->DecodeTempReg(readData);          // Convert read data into actual values
            readData    += 2 * (sizeof(uint8_t));   // Increment array pointer (by 2 bytes)
            size -= 2;
        }
        else
            return (this->Flt = DevFlt::UnRecognisedAddress);
    }

    return (this->Flt = DevFlt::None);      // Return no fault
}

AD741x::DevFlt AD741x::poleAvailability(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will check to see if the device is available via the I2C link.
 *************************************************************************************************/
    uint8_t i = 0;              // Variable used to loop through the number of available checks

    while (i != 10) {           // Only attempt to check device availability 10 times
        if (hal_I2C->poleDeviceRdy(this->I2CAddress) != I2CPeriph::DevFlt::None) {
            // Check to see if device is available. If device is not available then
            i++;                                // Go for another attempt
        }
        else {                                  // If "No Fault" is set
            return(this->Flt = DevFlt::None);   // Exit Function, and indicate no fault
        }
    }

    // If all 10 attempts have failed, then indicate a fault with device
    return(this->Flt = DevFlt::Fault);           // Return fault state
}

AD741x::DevFlt AD741x::poleConfigRead(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Configuration Register, and then copy contents into Class.
 *************************************************************************************************/
    uint8_t rData[1] = { 0 };                   // Array to contain the Temperature values

    uint32_t packetsize = 0;            // Variable to store the number of bytes to read/write

    if ( (this->Flt == DevFlt::Initialised) || (this->AddressPointer != AD741x_ConfigReg) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Configuration register.
        // Then request update to the Address pointer
        packetsize = this->UpdateAddressPointer(&rData[0], AD741x_ConfigReg);
            // Request Address Pointer update

        // Then transmit the updated request via I2C
        if ( hal_I2C->poleMasterTransmit(
                this->I2CAddress, &rData[0], packetsize) != I2CPeriph::DevFlt::None )
            return (this->Flt = DevFlt::Fault);
    }
    // Ensure the Read of this register is captured.
    this->AdBuff.InputWrite(
            this->AddressForm(AD741x_ConfigReg, Form::Read)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    // Then commence a read of the Temperature Registers
    if ( hal_I2C->poleMasterReceive(this->I2CAddress, &rData[0], 1) != I2CPeriph::DevFlt::None )
        return (this->Flt = DevFlt::Fault);

    return (this->deconstructData(&rData[0], 1));       // Decode data, and return any faults
                                                        // generated
}

AD741x::DevFlt AD741x::poleConfigWrite(I2CPeriph *hal_I2C,
                        PwrState Mode, FiltState Filt, OneShot Conv) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Configuration Register, and then copy contents into Class.
 *************************************************************************************************/
    uint8_t rData[2] = { 0 };                   // Array to contain the Temperature values

    uint32_t packetsize = 0;            // Variable to store the number of bytes to read/write

    packetsize = this->UpdateConfigReg(&rData[0], Mode, Filt, Conv);
        // Request a write of the Configuration register

    // Then transmit the updated request via I2C
    if ( hal_I2C->poleMasterTransmit(
            this->I2CAddress, &rData[0], packetsize) != I2CPeriph::DevFlt::None )
        return (this->Flt = DevFlt::Fault);

    return(this->Flt = DevFlt::None);   // Indicate no failures if none detected
}

AD741x::DevFlt AD741x::poleTempRead(I2CPeriph *hal_I2C) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Temperature Register, and then calculate the actual reading.
 *************************************************************************************************/
    uint8_t rData[2] = { 0 };           // Array to contain the Temperature values

    uint32_t packetsize = 0;            // Variable to store the number of bytes to read/write

    if ( (this->Flt == DevFlt::Initialised) || (this->AddressPointer != AD741x_TemperatureReg) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Temperature register.
        // Then request update to the Address pointer
        packetsize = this->UpdateAddressPointer(&rData[0], AD741x_TemperatureReg);
            // Request Address Pointer update

        // Then transmit the updated request via I2C
        if ( hal_I2C->poleMasterTransmit(
                this->I2CAddress, &rData[0], packetsize) != I2CPeriph::DevFlt::None )
            return (this->Flt = DevFlt::Fault);
    }
    // Ensure the Read of this register is captured.
    this->AdBuff.InputWrite(
            this->AddressForm(AD741x_TemperatureReg, Form::Read)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    // Then commence a read of the Temperature Registers
    if ( hal_I2C->poleMasterReceive(this->I2CAddress, &rData[0], 2) != I2CPeriph::DevFlt::None )
        return (this->Flt = DevFlt::Fault);

    return (this->deconstructData(&rData[0], 2));       // Decode data, and return any faults
                                                        // generated
}

void AD741x::reInitialise(void) {
/**************************************************************************************************
 * This function is to be called when the internal mechanics of the class need to be reset. This
 * is likely to occur in fault situations, or at initialisation.
 *************************************************************************************************/
    this->AdBuff.QFlush();                  // Clear the Address Pointer queue
    this->AddressPointer    = 0xFF;         // Initialise the Address pointer to 0xFF
                                            // This will be corrected on first transfer to device
    this->Flt           = DevFlt::Initialised;      // Set fault to initialised
}

void AD741x::intConfigRead(I2CPeriph *hal_I2C, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for the contents of the Configuration Register
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint8_t tempsize = 0;           // Variable to store the size of request

    if ( (this->Flt == DevFlt::Initialised) ||
         (this->LastAddresPointRqst() != AD741x_ConfigReg) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Temperature Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        tempsize = this->UpdateAddressPointer(&wBuff[this->wtcmpTarget], AD741x_ConfigReg);

        hal_I2C->intMasterReq(this->I2CAddress,
                              tempsize,
                              &wBuff[this->wtcmpTarget],
                              I2CPeriph::CommMode::AutoEnd, I2CPeriph::Request::START_WRITE,
                              &(this->I2CWFlt), &(this->wtcmpFlag));

        this->wtcmpTarget   += tempsize;    // Copy expected size to write complete target
    }
    // Ensure the Read of this register is captured.
    this->AdBuff.InputWrite(
            this->AddressForm(AD741x_ConfigReg, Form::Read)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    hal_I2C->intMasterReq(this->I2CAddress,
                          1,
                          &rBuff[this->rdcmpTarget],
                          I2CPeriph::CommMode::AutoEnd, I2CPeriph::Request::START_READ,
                          &(this->I2CRFlt), &(this->rdcmpFlag));

    this->rdcmpTarget   += 1;       // Put expected size of read back into read complete target
}

void AD741x::intConfigWrite(I2CPeriph *hal_I2C,
                            PwrState Mode, FiltState Filt, OneShot Conv, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for updating the contents of the Configuration Register
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint8_t tempsize = 0;           // Variable to store the size of request

    tempsize = this->UpdateConfigReg(&wBuff[this->wtcmpTarget], Mode, Filt, Conv);

    hal_I2C->intMasterReq(this->I2CAddress,
                          tempsize,
                          &wBuff[this->wtcmpTarget],
                          I2CPeriph::CommMode::AutoEnd, I2CPeriph::Request::START_WRITE,
                          &(this->I2CWFlt), &(this->wtcmpFlag));

    this->wtcmpTarget   += tempsize;// Copy expected size to write complete target
}

void AD741x::intTempRead(I2CPeriph *hal_I2C, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request of temperature read of the AD741x device.
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint8_t tempsize = 0;           // Variable to store the size of request

    if ( (this->Flt == DevFlt::Initialised) ||
         (this->LastAddresPointRqst() != AD741x_TemperatureReg) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Temperature Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        tempsize = this->UpdateAddressPointer(&wBuff[this->wtcmpTarget], AD741x_TemperatureReg);

        hal_I2C->intMasterReq(this->I2CAddress,
                              tempsize,
                              &wBuff[this->wtcmpTarget],
                              I2CPeriph::CommMode::AutoEnd, I2CPeriph::Request::START_WRITE,
                              &(this->I2CWFlt), &(this->wtcmpFlag));

        this->wtcmpTarget   += tempsize;    // Copy expected size to write complete target
    }
    // Ensure the Read of this register is captured.
    this->AdBuff.InputWrite(
            this->AddressForm(AD741x_TemperatureReg, Form::Read)
            );
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    hal_I2C->intMasterReq(this->I2CAddress,
                          2,
                          &rBuff[this->rdcmpTarget],
                          I2CPeriph::CommMode::AutoEnd, I2CPeriph::Request::START_READ,
                          &(this->I2CRFlt), &(this->rdcmpFlag));

    this->rdcmpTarget   += 2;       // Put expected size of read back into read complete target
}

void AD741x::intCheckCommStatus(uint8_t *rBuff, uint16_t size) {
/**************************************************************************************************
 * Function needs to be called periodically, as this is used to check status of the interrupt
 * based communication, and then deconstruct the data if there is any available.
 *************************************************************************************************/
    this->deconstructData(rBuff, size);         // Deconstruct data
}

AD741x::~AD741x()
{
    // TODO Auto-generated destructor stub
}

