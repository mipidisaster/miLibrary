/**************************************************************************************************
 * @file        BME280I2C.cpp
 * @author      Thomas
 * @brief       Source file for the BMR280 series of temperature sensors (I2C interface)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_BMEI2C_HD               // Header for the BME280 I2C interface
#include FilInd_BME280def               // Header for the BME280 definitions

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_BME280_HD               // Provide the lower level class for the BME280 device

#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class
#include FilInd_I2CPe__HD               // Include class for I2C Peripheral

//=================================================================================================

BME280I2C::BME280I2C(AddrBit ASPin, Form *FormArray, uint16_t FormSize)
/**************************************************************************************************
 * Generate a class instance of the BME280 device, which is to be communicated via I2C.
 * This will first of link up with the lower level class for the BME280 device, which defines the
 * functions and parameters to control/understand the device.
 * It will then generate specific instances for the I2C communication, i.e. Address.
 *************************************************************************************************/
    : BME280(FormArray, FormSize) {

    _address_pin_       = ASPin;        // Pass Address Pin into class parameters

    _i2c_address_       = 0x0000;       // Populate I2C address, with 0. Will be updated
                                        // correctly, by next function call
    getI2CAddress();    // Determine the I2C address from provided parameters

    i2c_wrte_flt      = I2CPeriph::DevFlt::kNone; // I2C write fault status set to "None"
    wrte_cmp_flg      = 0x00;                     // Initialise the communication complete flag
    wrte_cmp_target   = 0x00;                     // Initialise the target communication count

    i2c_read_flt      = I2CPeriph::DevFlt::kNone; // I2C read fault status set to "None"
    read_cmp_flag     = 0x00;                     // Initialise the communication complete flag
    read_cmp_target   = 0x00;                     // Initialise the target communication count
}

void BME280I2C::getI2CAddress(void) {
/**************************************************************************************************
 * Function will bring in the PartNumber and _address_pin_ provided to the class, and determine what
 * the I2C address is.
 * This is then put into the class "_i2c_address".
 *************************************************************************************************/
    if (_address_pin_ == AddrBit::kVdd) {       // If address is pulled to Vcc
        _i2c_address_ = BME280_I2CADDRSS_VDD;   // Use Vcc Address
    }
    else {                                      // If address pin is anything else, then set to
        _i2c_address_ = BME280_I2CADDRSS_GND;   // Gnd Address
    }
}

BME280I2C::DevFlt BME280I2C::poleAvailability(I2CPeriph *Interface) {
/**************************************************************************************************
 * Function will check to see if the device is available via the I2C link.
 *************************************************************************************************/
    uint8_t i = 0;              // Variable used to loop through the number of available checks

    while (i != 10) {           // Only attempt to check device availability 10 times
        if (Interface->poleDeviceRdy(_i2c_address_) != I2CPeriph::DevFlt::kNone) {
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

BME280I2C::DevFlt BME280I2C::poleCalibrateRead(I2CPeriph *Interface) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Calibration/Conversion Register(s), and then copy and decode them within
 * the Class.
 *************************************************************************************************/
    uint8_t rData[BME280_TEMP_CAL_COUNT + BME280_PRES_CAL_COUNT + 1 + 7]   = { 0 };
    uint8_t wData[3]    = { 0 };
    // Array to contain the read register contents, and the number of bytes to be read

    if ( (flt == DevFlt::kInitialised) || (_address_pointer_ != BME280_T1_LSB) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Calibration/Conversion register.
        // Then request update to the Address pointer
        if (Interface->poleMasterTransmit(  _i2c_address_,
                                            &wData[0],
                                            updateAddressPointer(&wData[0], BME280_T1_LSB))
                != I2CPeriph::DevFlt::kNone ) {
            return (flt = DevFlt::kFault);
        }
    }

    if (Interface->poleMasterReceive(   _i2c_address_,
                                        &rData[0],
                                        setupAddressAutoRead(
                                                BME280_T1_LSB,
                                                BME280_TEMP_CAL_COUNT + BME280_PRES_CAL_COUNT))
                    != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[1],
                                        updateAddressPointer(&wData[1], BME280_H1_8BIT))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    if (Interface->poleMasterReceive(   _i2c_address_,
                                        &rData[BME280_TEMP_CAL_COUNT + BME280_PRES_CAL_COUNT],
                                        setupAddressAutoRead(
                                                BME280_H1_8BIT,
                                                1))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[2],
                                        updateAddressPointer(&wData[2], BME280_H2_LSB))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    if (Interface->poleMasterReceive(   _i2c_address_,
                                        &rData[BME280_TEMP_CAL_COUNT + BME280_PRES_CAL_COUNT + 1],
                                        setupAddressAutoRead(
                                                BME280_H2_LSB,
                                                7))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    return(
      flt = deconstructData(&rData[0], BME280_TEMP_CAL_COUNT + BME280_PRES_CAL_COUNT + 1 + 7)
          );           // Return any fault(s) detected
}


BME280I2C::DevFlt BME280I2C::poleConfigRead(I2CPeriph *Interface) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Configuration Register(s), and then copy and decode them within the Class.
 *************************************************************************************************/
    uint8_t rData[4]    = { 0 };
    uint8_t wData[1]    = { 0 };

    if ( (flt == DevFlt::kInitialised) || (_address_pointer_ != BME280_CTRL_HUM) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Configuration register.
        // Then request update to the Address pointer
        if (Interface->poleMasterTransmit(  _i2c_address_,
                                            &wData[0],
                                            updateAddressPointer(&wData[0], BME280_CTRL_HUM))
                != I2CPeriph::DevFlt::kNone ) {
            return (flt = DevFlt::kFault);
        }
    }

    if (Interface->poleMasterReceive(   _i2c_address_,
                                        &rData[0],
                                        setupAddressAutoRead(
                                                BME280_CTRL_HUM,
                                                4))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    return(
      flt = deconstructData(&rData[0], 4)
          );           // Return any fault(s) detected
}

BME280I2C::DevFlt BME280I2C::poleConfigWrite(I2CPeriph *Interface,
                       ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                       ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant) {
/**************************************************************************************************
 * Function will do a direct transmit write to the device via I2C link (poling mode).
 *************************************************************************************************/
    uint8_t wData[6] = { 0 };

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[0],
                                        updateConfigReg(&wData[0],
                                                humd_config, pres_config, temp_config,
                                                sensor_mode,
                                                t_standby,
                                                filter_constant))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }


    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

BME280I2C::DevFlt BME280I2C::poleSensorRead(I2CPeriph *Interface) {
/**************************************************************************************************
 * Function will do a direct transmit and receive from device via I2C link (poling mode). Reading
 * the contents of the Sensor reading Register(s), and then copy and decode them within the Class.
 *************************************************************************************************/
    uint8_t rData[8]    = { 0 };
    uint8_t wData[1]    = { 0 };

    if ( (flt == DevFlt::kInitialised) || (_address_pointer_ != BME280_PRES) ) {
        // If Device class has just been created (fault = Initialised), or Address Pointer is not
        // equal to Sensor reading register.
        // Then request update to the Address pointer
        if (Interface->poleMasterTransmit(  _i2c_address_,
                                            &wData[0],
                                            updateAddressPointer(&wData[0], BME280_PRES))
                != I2CPeriph::DevFlt::kNone ) {
            return (flt = DevFlt::kFault);
        }
    }

    if (Interface->poleMasterReceive(   _i2c_address_,
                                        &rData[0],
                                        setupAddressAutoRead(
                                                BME280_PRES,
                                                8)
                                )
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }

    flt = deconstructData(&rData[0], 8);
    calculateSensorReadings();

    return (flt);       // Return any fault(s) detected
}

BME280I2C::DevFlt BME280I2C::poleRecommendedConfigWeather (I2CPeriph *Interface) {
/**************************************************************************************************
 * Recommended settings
 *************************************************************************************************/
    uint8_t wData[6] = { 0 };

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[0],
                                        updateConfigReg(&wData[0],
                                                BME280_RECOMMENDED_SETTING_WEATHER))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }


    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

BME280I2C::DevFlt BME280I2C::poleRecommendedConfigHumidity(I2CPeriph *Interface) {
/**************************************************************************************************
 * Recommended settings
 *************************************************************************************************/
    uint8_t wData[6] = { 0 };

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[0],
                                        updateConfigReg(&wData[0],
                                                BME280_RECOMMENDED_SETTING_HUMIDTY))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }


    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

BME280I2C::DevFlt BME280I2C::poleRecommendedConfigIndoor  (I2CPeriph *Interface) {
/**************************************************************************************************
 * Recommended settings
 *************************************************************************************************/
    uint8_t wData[6] = { 0 };

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[0],
                                        updateConfigReg(&wData[0],
                                                BME280_RECOMMENDED_SETTING_INDOOR))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }


    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

BME280I2C::DevFlt BME280I2C::poleRecommendedConfigGaming  (I2CPeriph *Interface) {
/**************************************************************************************************
 * Recommended settings
 *************************************************************************************************/
    uint8_t wData[6] = { 0 };

    if (Interface->poleMasterTransmit(  _i2c_address_,
                                        &wData[0],
                                        updateConfigReg(&wData[0],
                                                BME280_RECOMMENDED_SETTING_GAMING))
            != I2CPeriph::DevFlt::kNone ) {
        return (flt = DevFlt::kFault);
    }


    return(flt = DevFlt::kNone);   // Indicate no failures if none detected
}

void BME280I2C::reInitialise(void) {
/**************************************************************************************************
 * This function is to be called when the internal mechanics of the class need to be reset. This
 * is likely to occur in fault situations, or at initialisation.
 *************************************************************************************************/
    popGenParam();
}

void BME280I2C::intCalibrateRead(I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for the contents of the Calibration Register(s)
 *    > Does this in 3 passes - Temperature, Pressure, Humidity
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;         // Variable to store the size of request

    if      ( (_calibration_points_ & BME280_TEMP_CAL_MASK) != BME280_TEMP_CAL_MASK ) {
        if  ( (flt == DevFlt::kInitialised) || (lastAddresPointRqst() != BME280_T1_LSB) ) {
            /* If the class has just been constructed or the Last Address Pointer request is not
             * equal to the "Calibration/Conversion Register"
             * Then need to generate a I2C Form to write a update to the Address pointer
             */
            temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_T1_LSB);

            Interface->intMasterReq(_i2c_address_,
                                    temp_size,
                                    &wBuff[wrte_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Write,
                                    &(i2c_wrte_flt), &(wrte_cmp_flg));

            wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
        }

        // Ensure the Read of this register is captured.
        temp_size = setupAddressAutoRead(BME280_T1_LSB,  BME280_TEMP_CAL_COUNT);
        // Ensure it is captured within the queue, and set to READ. Such that the decode will
        // check for any read backs

        Interface->intMasterReq(    _i2c_address_,
                                    temp_size,
                                    &rBuff[read_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Read,
                                    &(i2c_read_flt), &(read_cmp_flag));

        read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                            // target
    }

    else if ( (_calibration_points_ & BME280_PRES_CAL_MASK) != BME280_PRES_CAL_MASK ) {
        if  ( (flt == DevFlt::kInitialised) || (lastAddresPointRqst() != BME280_P1_LSB) ) {
            /* If the class has just been constructed or the Last Address Pointer request is not
             * equal to the "Calibration/Conversion Register".
             * Then need to generate a I2C Form to write a update to the Address pointer
             */
            temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_P1_LSB);

            Interface->intMasterReq(_i2c_address_,
                                    temp_size,
                                    &wBuff[wrte_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Write,
                                    &(i2c_wrte_flt), &(wrte_cmp_flg));

            wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
        }

        // Ensure the Read of this register is captured.
        temp_size = setupAddressAutoRead(BME280_P1_LSB,  BME280_PRES_CAL_COUNT);
        // Ensure it is captured within the queue, and set to READ. Such that the decode will
        // check for any read backs

        Interface->intMasterReq(    _i2c_address_,
                                    temp_size,
                                    &rBuff[read_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Read,
                                    &(i2c_read_flt), &(read_cmp_flag));

        read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                            // target
    }

    else if ( (_calibration_points_ & BME280_HUMD_CAL_MASK) != BME280_HUMD_CAL_MASK ) {
        if  ( (flt == DevFlt::kInitialised) || (lastAddresPointRqst() != BME280_H1_8BIT) ) {
            /* If the class has just been constructed or the Last Address Pointer request is not
             * equal to the "Calibration/Conversion Register".
             * Then need to generate a I2C Form to write a update to the Address pointer
             */
            temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_H1_8BIT);

            Interface->intMasterReq(_i2c_address_,
                                    temp_size,
                                    &wBuff[wrte_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Write,
                                    &(i2c_wrte_flt), &(wrte_cmp_flg));

            wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
        }

        // Ensure the Read of this register is captured.
        temp_size = setupAddressAutoRead(BME280_H1_8BIT,  1);
        // Ensure it is captured within the queue, and set to READ. Such that the decode will
        // check for any read backs

        Interface->intMasterReq(    _i2c_address_,
                                    temp_size,
                                    &rBuff[read_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Read,
                                    &(i2c_read_flt), &(read_cmp_flag));

        read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                            // target
        ///////////////////////////////////////////////////////////////////////////////////////////
        temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_H2_LSB);

        Interface->intMasterReq(    _i2c_address_,
                                    temp_size,
                                    &wBuff[wrte_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Write,
                                    &(i2c_wrte_flt), &(wrte_cmp_flg));

        wrte_cmp_target   += temp_size;     // Copy expected size to write complete target

        // Ensure the Read of this register is captured.
        temp_size = setupAddressAutoRead(BME280_H2_LSB,  7);
        // Ensure it is captured within the queue, and set to READ. Such that the decode will
        // check for any read backs

        Interface->intMasterReq(    _i2c_address_,
                                    temp_size,
                                    &rBuff[read_cmp_target],
                                    I2CPeriph::CommMode::kAutoEnd,
                                    I2CPeriph::Request::kStart_Read,
                                    &(i2c_read_flt), &(read_cmp_flag));

        read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                            // target
    }
}

void BME280I2C::intConfigRead(I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for the contents of the Configuration Register(s)
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;         // Variable to store the size of request

    if  ( (flt == DevFlt::kInitialised) || (lastAddresPointRqst() != BME280_CTRL_HUM) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Configuration Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_CTRL_HUM);

        Interface->intMasterReq(_i2c_address_,
                                temp_size,
                                &wBuff[wrte_cmp_target],
                                I2CPeriph::CommMode::kAutoEnd,
                                I2CPeriph::Request::kStart_Write,
                                &(i2c_wrte_flt), &(wrte_cmp_flg));

        wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
    }

    // Ensure the Read of this register is captured.
    temp_size = setupAddressAutoRead(BME280_CTRL_HUM,  4);
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    Interface->intMasterReq(    _i2c_address_,
                                temp_size,
                                &rBuff[read_cmp_target],
                                I2CPeriph::CommMode::kAutoEnd,
                                I2CPeriph::Request::kStart_Read,
                                &(i2c_read_flt), &(read_cmp_flag));

    read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                        // target
}

void BME280I2C::intConfigWrite(I2CPeriph *Interface, uint8_t *wBuff,
                       ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                       ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant) {
/**************************************************************************************************
 * Interrupt based request for the populating contents of the Configuration Register(s)
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;          // Variable to store the size of request

    temp_size = updateConfigReg(&wBuff[0],
                                humd_config, pres_config, temp_config,
                                sensor_mode,
                                t_standby,
                                filter_constant);

    Interface->intMasterReq(    _i2c_address_,
                                temp_size,
                                &wBuff[wrte_cmp_target],
                                I2CPeriph::CommMode::kAutoEnd,
                                I2CPeriph::Request::kStart_Write,
                                &(i2c_wrte_flt), &(wrte_cmp_flg));

    wrte_cmp_target   += temp_size; // Copy expected size to write complete target
}

void BME280I2C::intSensorRead(I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request for the contents of the Sensor reading Register(s)
 *
 * Note, that the I2C device will handle the communication - this requires the use of the I2C
 * form type (scoped from the I2CPeriph class)
 *************************************************************************************************/
    uint16_t temp_size = 0;         // Variable to store the size of request

    if  ( (flt == DevFlt::kInitialised) || (lastAddresPointRqst() != BME280_PRES) ) {
        /* If the class has just been constructed or the Last Address Pointer request is not
         * equal to the "Sensor reading Register".
         * Then need to generate a I2C Form to write a update to the Address pointer
         */
        temp_size = updateAddressPointer(&wBuff[wrte_cmp_target], BME280_PRES);

        Interface->intMasterReq(_i2c_address_,
                                temp_size,
                                &wBuff[wrte_cmp_target],
                                I2CPeriph::CommMode::kAutoEnd,
                                I2CPeriph::Request::kStart_Write,
                                &(i2c_wrte_flt), &(wrte_cmp_flg));

        wrte_cmp_target   += temp_size;     // Copy expected size to write complete target
    }

    // Ensure the Read of this register is captured.
    temp_size = setupAddressAutoRead(BME280_PRES,  8);
    // Ensure it is captured within the queue, and set to READ. Such that the decode will
    // check for any read backs

    Interface->intMasterReq(    _i2c_address_,
                                temp_size,
                                &rBuff[read_cmp_target],
                                I2CPeriph::CommMode::kAutoEnd,
                                I2CPeriph::Request::kStart_Read,
                                &(i2c_read_flt), &(read_cmp_flag));

    read_cmp_target   += temp_size;     // Put expected size of read back into read complete
                                        // target
}

void BME280I2C::intRecommendedConfigWeather (I2CPeriph *Interface, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request -- Recommended settings
 *************************************************************************************************/
    intConfigWrite(Interface, wBuff, BME280_RECOMMENDED_SETTING_WEATHER);
}

void BME280I2C::intRecommendedConfigHumidity(I2CPeriph *Interface, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request -- Recommended settings
 *************************************************************************************************/
    intConfigWrite(Interface, wBuff, BME280_RECOMMENDED_SETTING_HUMIDTY);
}

void BME280I2C::intRecommendedConfigIndoor  (I2CPeriph *Interface, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request -- Recommended settings
 *************************************************************************************************/
    intConfigWrite(Interface, wBuff, BME280_RECOMMENDED_SETTING_INDOOR);
}

void BME280I2C::intRecommendedConfigGaming  (I2CPeriph *Interface, uint8_t *wBuff) {
/**************************************************************************************************
 * Interrupt based request -- Recommended settings
 *************************************************************************************************/
    intConfigWrite(Interface, wBuff, BME280_RECOMMENDED_SETTING_GAMING);
}

uint8_t BME280I2C::intCheckCommStatus(uint8_t *rBuff) {
/**************************************************************************************************
 * Function needs to be called periodically, as this is used to check status of the interrupt
 * based communication, and then deconstruct the data if there is any available.
 *
 * Will return "1" if all the requested data has been read/written, and that this has been
 * deconstructed into the class
 *************************************************************************************************/
    if ( (read_cmp_target == read_cmp_flag) && (wrte_cmp_target == wrte_cmp_flg) ) {
        deconstructData(rBuff, read_cmp_target);    // Deconstruct data
        return (1);
    }

    return (0);
}

void BME280I2C::clearCommunicationCount(void) {
/**************************************************************************************************
 * Function will reset the write and read interrupt fault flags and target/ counts.
 *************************************************************************************************/
    wrte_cmp_target = 0;        // Clear the communication write target count
    read_cmp_target = 0;        // Clear the communication  read target count

    wrte_cmp_flg    = 0;        // Clear the communication write flag (actual count)
    read_cmp_flag   = 0;        // Clear the communication  read flag (actual count)
}

uint16_t BME280I2C::readI2CAddress(void) {
/**************************************************************************************************
 * Basic function to return the calculated I2C address of the target device
 *************************************************************************************************/
    return (_i2c_address_);
}

BME280I2C::~BME280I2C()
{
    // TODO Auto-generated destructor stub
}

