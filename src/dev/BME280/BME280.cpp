/**************************************************************************************************
 * @file        BME280.cpp
 * @author      Thomas
 * @brief       Source file for the BME280 series of temperature sensors
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_BME280_HD               // Header for BME280 Device
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
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================

void BME280::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the BME280 class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    _mode_          = ctrl_mode::kSleep;            // Default to "Sleep" mode
    _filt_          = cfig_filt::kOff;              // Default to filter disabled
    _sample_rate_   = t_sb::ktsmp_0_5ms;            // Default to 0.5ms

    _hum_oversamp_  = ctrl_hum::kSkipped;           // Default to skipped
    _prs_oversamp_  = osrs_p::kSkipped;             //
    _tmp_oversamp_  = osrs_t::kSkipped;             //

    _address_pointer_    = 0xFF;        // Initialise the Address pointer to 0xFF
                                        // This will be corrected on first transfer to device

    _calibration_data_      = { 0 };                // Initialise everything to zero
    _calibration_points_    = 0;

    flt                 = DevFlt::kInitialised;
    _temp_raw_          = 0;                        //
    _pres_raw_          = 0;                        //
    _humd_raw_          = 0;                        //

    _t_fine_        = 0;                            //
    temperature     = 0.00f;                        //
    pressure        = 0.00f;                        //
    humidity        = 0.00f;                        //
}

BME280::BME280(Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * Generate the "BME280" class, and populate everything with the state of the device at power up
 *************************************************************************************************/
    popGenParam();

    _address_buff_.create(FormArray, FormSize);     // Create GenBuffer internal Address
}

uint8_t BME280::parseRegisters(uint8_t *buff, uint8_t address) {
/**************************************************************************************************
 * Take input data and put into the class internals for the register(s) data.
 * "*buff" is an array, with the "address" indicating what the first position of this array equates
 * to within the BME280 device...
 *   i.e.   buff[0]/[1]   ->  address would be 0x88 and 0x89 for the points of "dig_T1"
 *
 * This realise upon upper functionality to put the correct initial data point into "*buff" for
 * the parsing.
 *
 * Will return the number of bytes that have been read. So that can be used as part of a while
 * loop scanning through an array of data.
 * So in the above example (0x88), as 2 bytes are used it would provide a value of "2".
 *
 * If the input 'address' is not one of the addresses for calibration data, output will default to
 * "1"
 *************************************************************************************************/
    switch (address) {
    // Temperature components
    ///////////////////////////////
        case    BME280_T1_LSB:
            _calibration_data_.dig_T1  =  (uint16_t) (  ((uint16_t) buff[1] << 8) |
                                                         (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 0);

            return 2;
            break;
        case    BME280_T2_LSB:
            _calibration_data_.dig_T2  =  ( int16_t) (  ((uint16_t) buff[1] << 8) |
                                                         (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 1);

            return 2;
            break;
        case    BME280_T3_LSB:
            _calibration_data_.dig_T3  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 2);

            return 2;
            break;

    // Pressure components
    ///////////////////////////////
        case    BME280_P1_LSB:
            _calibration_data_.dig_P1  =  (uint16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 3);

            return 2;
            break;
        case    BME280_P2_LSB:
            _calibration_data_.dig_P2  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 4);

            return 2;
            break;
        case    BME280_P3_LSB:
            _calibration_data_.dig_P3  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 5);

            return 2;
            break;
        case    BME280_P4_LSB:
            _calibration_data_.dig_P4  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 6);

            return 2;
            break;
        case    BME280_P5_LSB:
            _calibration_data_.dig_P5  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 7);

            return 2;
            break;
        case    BME280_P6_LSB:
            _calibration_data_.dig_P6  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 8);

            return 2;
            break;
        case    BME280_P7_LSB:
            _calibration_data_.dig_P7  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 9);

            return 2;
            break;
        case    BME280_P8_LSB:
            _calibration_data_.dig_P8  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 10);
            return 2;
            break;
        case    BME280_P9_LSB:
            _calibration_data_.dig_P9  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 11);

            return 2;
            break;

    // Humidty components
    ///////////////////////////////
        case    BME280_H1_8BIT:
            _calibration_data_.dig_H1  =  ( int8_t) buff[0];
            _calibration_points_   |= (1 << 12);

            return 1;
            break;
        case    BME280_H2_LSB:
            _calibration_data_.dig_H2  =  ( int16_t)  (  ((uint16_t) buff[1] << 8) |
                                                          (uint16_t) buff[0]  );
            _calibration_points_   |= (1 << 13);

            return 2;
            break;
        case    BME280_H3_8BIT:
            _calibration_data_.dig_H3  =  ( int8_t) buff[0];
            _calibration_points_   |= (1 << 14);

            return 1;
            break;
        case    BME280_H45_3bytes:
            _calibration_data_.dig_H4  =  ( int16_t) ( ( buff[0] << 4 ) | ( buff[1] & 0x0F ) );
            _calibration_data_.dig_H5  =  ( int16_t) ( ( buff[2] << 4 ) | ( buff[1] >> 4 ) );
            _calibration_points_   |= (3 << 15);

            return 3;
            break;
        case    BME280_H6_8BIT:
            _calibration_data_.dig_H6  =  ( int8_t) buff[0];
            _calibration_points_   |= (1 << 17);

            return 1;
            break;

    // Configuration components
    ///////////////////////////////
        case    BME280_CTRL_HUM:
            _hum_oversamp_  = (ctrl_hum) (buff[0] & BME280_CTRL_HUM_MASK);

            return 1;
            break;
        case    BME280_STTS:

            return 1;
            break;
        case    BME280_CTRL_MEAS:
            _tmp_oversamp_  = (osrs_t)      (buff[0] & BME280_CTRL_T_MASK);
            _prs_oversamp_  = (osrs_p)      (buff[0] & BME280_CTRL_P_MASK);
            _mode_          = (ctrl_mode)   (buff[0] & BME280_CTRL_MODE_MASK);

            return 1;
            break;
        case    BME280_CFIG:
            _sample_rate_   = (t_sb)        (buff[0] & BME280_CFIG_TSMP_MASK);
            _filt_          = (cfig_filt)   (buff[0] & BME280_CFIG_FILT_MASK);

            return 1;
            break;

    // Sensor data components
    ///////////////////////////////
        case    BME280_PRES:
            _pres_raw_      = (uint32_t)    (buff[0] << 12);
            _pres_raw_     |= (uint32_t)    (buff[1] << 4);
            _pres_raw_     |= (uint32_t)    (buff[2] >> 4);

            return 3;
            break;
        case    BME280_TEMP:
            _temp_raw_      = (uint32_t)    (buff[0] << 12);
            _temp_raw_     |= (uint32_t)    (buff[1] << 4);
            _temp_raw_     |= (uint32_t)    (buff[2] >> 4);

            return 3;
            break;
        case    BME280_HUMD:
            _humd_raw_      = (uint32_t)    (buff[0] << 8);
            _humd_raw_     |= (uint32_t)    (buff[1]);

            return 2;
            break;

    default: return 1; break;
    }

    // If get to the point, its an error. But will return 1
}

BME280::Form BME280::addressForm(uint8_t newAdd, BME280::Form::Dir Direction, uint8_t packetSize) {
/**************************************************************************************************
 * As class uses a Form structure, this function retrieves the input parameters and will then
 * output a Form structure specific for the BME280.
 *************************************************************************************************/
    Form new_form = { 0 };

    new_form.AddrPoint  = newAdd;
    new_form.ReadWrite  = Direction;
    new_form.packetSize = packetSize;

    return(new_form);
}

uint8_t BME280::updateAddressPointer(uint8_t *buff, uint8_t newval) {
/**************************************************************************************************
 * Function will put the new Address Pointer request into the input Buffer.
 * Buffer needs to be added, so as to allow for use of other buffers other than the internal
 * write buffer.
 *  Will also up in the new requested value into the "Address Pointer Buffer", so as to ensure
 *  that on time of read back, the class INTERNAL ADDRESS POINTER copy aligns up.
 *
 * Returns the number of bytes that need to be written (which will always be 1)
 *************************************************************************************************/
    _address_buff_.inputWrite(addressForm(newval, Form::kWrite, 0));
        // Packet size isn't used so doesn't need to be populated
        // Forced to '0' to ensure nothing gets read

    *(buff) = newval;           // Update the input buffer with new entry

    return (1);                 // Return number of bytes to be written to device comms
}

uint8_t BME280::setupAddressAutoRead(uint8_t address, uint8_t numberregisters) {
/**************************************************************************************************
 * Function will put in a new Address pointer request.
 * However will capture this as a READ (not write), and record the number of registers to be read;
 * this will need to be the same size as what is actually read from target device.
 * This will then allow the 'deconstructData' function to correctly auto-increment the registers
 *************************************************************************************************/
    _address_buff_.inputWrite(addressForm(address, Form::kRead, numberregisters));

    return (numberregisters);   // Return number of bytes to be written to device comms
}

uint8_t BME280::lastAddresPointRqst(void) {
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

    return(_address_buff_.pa[last_entry].AddrPoint +
           _address_buff_.pa[last_entry].packetSize);
    // Return the last Address request, and take into account the auto-increment of any reads
    //  Writes will be populated with a 'packetSize' of zer.
    //  Also no need to account for overrunning, as parameters are all uint8_t
}

uint8_t BME280::updateConfigReg(uint8_t *buff,
                                ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                                ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant) {
/**************************************************************************************************
 * Function will put the new configuration write request(s) into the input "buffer", and update
 * the AddressPointer with the last address that is written
 *
 * Returns the number of bytes that need to be written (which will always be 6)
 *************************************************************************************************/
    updateAddressPointer(buff, BME280_CFIG);    // The last register which is read is the "config"

    buff[0]  =  BME280_CTRL_HUM;
    buff[1]  = (uint8_t) humd_config;

    buff[2] = BME280_CTRL_MEAS;
    buff[3] = ( (uint8_t) pres_config ) | ( (uint8_t) temp_config ) | ( (uint8_t) sensor_mode );

    buff[4] = BME280_CFIG;
    buff[5] = ( (uint8_t) t_standby ) | ( (uint8_t) filter_constant );

    return(6);
}

BME280::DevFlt BME280::deconstructData(uint8_t *readData, uint16_t size) {
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

            _address_pointer_ = temp_form.AddrPoint;        // Copy Address contents to current
                                                            // Address
            if (temp_form.ReadWrite == Form::kWrite) { continue; }
                // If the form states that the communication was a "Write", then no data to
                // read back. So ignore this request.
        }
        else {  // If no entry in queue (but function still expecting to decode data), then
                // indicate Synchronised Error
            return (flt = DevFlt::kSynchronize_Error);
        }

        uint16_t packetNum  = 0;    // Variable for ensuring the correct data points are read from
                                    // input data
        while (packetNum  != temp_form.packetSize) {
            uint8_t step = 1;       // Variable to be used to ensure that packetNum will always
                                    // increase by one, so as to exit loop

            // Register is...
            //  -- CHIP_ID
            //====================================================================================
            if      (_address_pointer_ == BME280_ID)        {   // If the Address is the "Chip ID"
                 if ( *readData != 0x60 ) { return (flt = DevFlt::kChipId_Wrong); }
                 //size       -= 1; No need to increase the packetNum as this will be done
                 // automatically
            }

            //  -- Calibration point(s)
            //====================================================================================
            else if (__BME280_DATA_REGISTERS(_address_pointer_))   {
                step = parseRegisters(readData, _address_pointer_);
            }

            else
                return (flt = DevFlt::kUnrecognised_Address);

            packetNum           += step;
            _address_pointer_   += step;    // to account for the auto register increase
            readData            += step * sizeof(uint8_t);
        }

        // Ensure that the overall loop is being updated as well.
        size -= temp_form.packetSize;
    }

    return (flt = DevFlt::kNone);      // Return no fault
}

uint8_t BME280::deviceCalibrated(void) {
/**************************************************************************************************
 * Return "1" if all the calibration data points have been read. Otherwise return "0"
 *************************************************************************************************/
    return (
    (_calibration_points_ == (BME280_HUMD_CAL_MASK | BME280_PRES_CAL_MASK | BME280_TEMP_CAL_MASK))
    ? 1 : 0
           );
}

void  BME280::calculateSensorReadings(void) {
/**************************************************************************************************
 * Check to see if there is any faults with the device, and then determine which sensor readings
 * have been enabled, and then call the respective compensation function
 *************************************************************************************************/
    if (flt == DevFlt::kNone) {
        // If any of the sensor parameters are enabled, then first calculate Temperature
        if ( (_tmp_oversamp_ != osrs_t::kSkipped)       || (_prs_oversamp_ != osrs_p::kSkipped) ||
             (_hum_oversamp_ != ctrl_hum::kSkipped)) {
            compensateTemp();
        }

        if (_prs_oversamp_ != osrs_p::kSkipped)     {  compensatePres();  }

        if (_hum_oversamp_ != ctrl_hum::kSkipped)   {  compensateHumd();  }
    }
}

#ifdef BME280_FLOAT_ENABLE

void BME280::compensateTemp(void) {
/**************************************************************************************************
 * Calculate the temperature of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_temp_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * Floating point version of the Temperature compensation
 *************************************************************************************************/
    float var1;
    float var2;
    //float temperature;
    float temperature_min = -40.0f;
    float temperature_max = 85.0f;

    var1 = ((float) _temp_raw_) / 16384.0 - ((float) _calibration_data_.dig_T1) / 1024.0;
    var1 = var1 * ((float) _calibration_data_.dig_T2);
    var2 = (((float) _temp_raw_) / 131072.0 - ((float) _calibration_data_.dig_T1) / 8192.0);
    var2 = (var2 * var2) * ((float) _calibration_data_.dig_T3);
    _t_fine_ = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    //return temperature;
}

void BME280::compensatePres(void) {
/**************************************************************************************************
 * Calculate the pressure of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_press_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * Floating point version of the Pressure compensation
 *************************************************************************************************/
    float var1;
    float var2;
    float var3;
    //float pressure;
    float pressure_min = 30000.0;
    float pressure_max = 110000.0;

    var1 = ((float) _t_fine_ / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float) _calibration_data_.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((float) _calibration_data_.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((float) _calibration_data_.dig_P4) * 65536.0);
    var3 = ((float) _calibration_data_.dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((float) _calibration_data_.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((float) _calibration_data_.dig_P1);

    /* avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (float) _pres_raw_;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((float) _calibration_data_.dig_P9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((float) _calibration_data_.dig_P8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((float) _calibration_data_.dig_P7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    //return pressure;
}

void BME280::compensateHumd(void) {
/**************************************************************************************************
 * Calculate the humidity of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_humd_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * Floating point version of the Humidity compensation
 *************************************************************************************************/
    //float humidity;
    float humidity_min = 0.0;
    float humidity_max = 100.0;
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    float var6;

    var1 = ((float) _t_fine_) - 76800.0;
    var2 = (((float) _calibration_data_.dig_H4) * 64.0 +
           (((float) _calibration_data_.dig_H5) / 16384.0) * var1);

    var3 = _humd_raw_ - var2;
    var4 = ((float) _calibration_data_.dig_H2) / 65536.0;
    var5 = (1.0 + (((float) _calibration_data_.dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((float) _calibration_data_.dig_H6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((float) _calibration_data_.dig_H1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    //return humidity;
}

#else

void BME280::compensateTemp(void) {
/**************************************************************************************************
 * Calculate the temperature of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_temp_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * None floating point version of the Temperature compensation (32/64bit)
 *************************************************************************************************/
    int32_t var1;
    int32_t var2;
    //int32_t temperature;
    int32_t _temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((_temp_raw_ / 8) - ((int32_t) _calibration_data_.dig_T1 * 2));
    var1 = (var1 * ((int32_t) _calibration_data_.dig_T2)) / 2048;
    var2 = (int32_t)((_temp_raw_ / 16) - ((int32_t) _calibration_data_.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t) _calibration_data_.dig_T3)) / 16384;
    _t_fine_ = var1 + var2;
    _temperature = (_t_fine_ * 5 + 128) / 256;

    if (_temperature < temperature_min)
    {
        _temperature = temperature_min;
    }
    else if (_temperature > temperature_max)
    {
        _temperature = temperature_max;
    }

    temperature = (float) ( _temperature * 0.01f );     // Correct the temperature to degC

    //return temperature;
}

#ifndef BME280_32BIT_ENABLE /* 64 bit compensation for pressure data */

void BME280::compensatePres(void) {
/**************************************************************************************************
 * Calculate the pressure of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_press_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * None floating point version of the Temperature compensation (64bit version)
 *************************************************************************************************/
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t _pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t) _t_fine_) - 128000;
    var2 = var1 * var1 * (int64_t) _calibration_data_.dig_P6;
    var2 = var2 + ((var1 * (int64_t) _calibration_data_.dig_P5) * 131072);
    var2 = var2 + (((int64_t) _calibration_data_.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t) _calibration_data_.dig_P3) / 256) +
                 ((var1 * ((int64_t) _calibration_data_.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t) _calibration_data_.dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - _pres_raw_;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t) _calibration_data_.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t) _calibration_data_.dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t) _calibration_data_.dig_P7) * 16);
        _pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (_pressure < pressure_min)
        {
            _pressure = pressure_min;
        }
        else if (_pressure > pressure_max)
        {
            _pressure = pressure_max;
        }
    }
    else
    {
        _pressure = pressure_min;
    }

    pressure = ( (float) _pressure * 0.01f );           // Correct the pressure to Pa

    //return pressure;
}

#else                       /* 32 bit compensation for pressure data */

void BME280::compensatePres(void) {
/**************************************************************************************************
 * Calculate the pressure of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_press_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * None floating point version of the Temperature compensation (32bit version)
 *************************************************************************************************/
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t _pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t) _t_fine_) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) _calibration_data_.dig_P6);
    var2 = var2 + ((var1 * ((int32_t) _calibration_data_.dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t) _calibration_data_.dig_P4) * 65536);
    var3 = ( _calibration_data_.dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t) _calibration_data_.dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t) _calibration_data_.dig_P1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - _pres_raw_;
        _pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (_pressure < 0x80000000)
        {
            _pressure = (_pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            _pressure = (_pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t) _calibration_data_.dig_P9) * ((int32_t)(((_pressure / 8) *
                 (_pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(_pressure / 4)) * ((int32_t) _calibration_data_.dig_P8)) / 8192;
        _pressure = (uint32_t)((int32_t)_pressure + ((var1 + var2 +
                  _calibration_data_.dig_P7) / 16));

        if (_pressure < pressure_min)
        {
            _pressure = pressure_min;
        }
        else if (_pressure > pressure_max)
        {
            _pressure = pressure_max;
        }
    }
    else
    {
        _pressure = pressure_min;
    }

    pressure = ( (float) _pressure * 1.00f );           // Correct the pressure to Pa

    //return pressure;
}

#endif

void BME280::compensateHumd(void) {
/**************************************************************************************************
 * Calculate the humidity of the BME280 converted data. Doesn't receive any input, as will be
 * using the internal parameter "_humd_raw_" to calculate on - this is therefore expected to be
 * populated when called
 * None floating point version of the Humidity compensation (32/64bit)
 *************************************************************************************************/
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t _humidity;
    uint32_t humidity_max = 102400;

    var1 = _t_fine_ - ((int32_t)76800);
    var2 = (int32_t)(_humd_raw_ * 16384);
    var3 = (int32_t)(((int32_t) _calibration_data_.dig_H4) * 1048576);
    var4 = ((int32_t) _calibration_data_.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t) _calibration_data_.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t) _calibration_data_.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t) _calibration_data_.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t) _calibration_data_.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    _humidity = (uint32_t)(var5 / 4096);

    if (_humidity > humidity_max)
    {
        _humidity = humidity_max;
    }

    humidity = (float) ( _humidity * (1.0f / 1024.0f) );// Correct the temperature to degC

    //return humidity;
}

#endif

BME280::~BME280() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
    // TODO Auto-generated destructor stub
}

