/**************************************************************************************************
 * @file        BME280.h
 * @author      Thomas
 * @brief       Header file for the BMR280 series of temperature sensors
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communication between the target embedded device and the
 * selected BME280 "Combined humidity and pressure sensor".
 *
 *      This is the core functionality of the BME280 class, and handles the basic management of
 *      the device. To facilitate communicate to the device - either via I2C or SPI, the
 *      respective "add-on" of the class needs to be used instead of this one.
 *          However, this class can be used by itself so long as the communication is handled via
 *          external means.
 *
 *      Fundamental functions used within this class, which are protected, so not visible outside
 *      the class:
 *          ".addressForm"          - Used to generate the BME280 Form structure, used to handle
 *                                    what the state of device's Address Pointer has been set too
 *                                    (see section below on BME280 Form)
 *          ".parseRegisters"       - Recognises what each register read from BME280 is, and will
 *                                    parse this data and copy into the internals of class
 *          ".compensateTemp"/".compensatePres"/".compensateHumd"
 *                                  - Will read the class internal parameters "_temp_raw_",
 *                                    "_pres_raw_" and "_humd_raw_", and calculate the actual
 *                                    values.
 *              >> CONFIGURATION OPTIONS <<
 *                  #define BME280_FLOAT_ENABLE
 *                      If this is defined, then will calculate the Temperature/Pressure/Humidity
 *                      making use of "float".
 *                          Based upon the BME280 github page
 *                          "https://github.com/BoschSensortec/BME280_driver", this should really
 *                          be "double", however doesn't appear to work that well using a
 *                          RaspberryPi. Possibility for further improvement
 *
 *                  #define BME280_32BIT_ENABLE
 *                      If this is defined (and NOT BME280_FLOAT_ENABLE"), then will calculate
 *                      Temperature/Pressure/Humidity through the use of 32bit variables only.
 *                      This may result in some deviations against the actual reading, however
 *                      output will be a floating point value
 *
 *                  Default configuration
 *                      The default configuration is to make use of 32bit for Temperature and
 *                      Humidity. The pressure will be calculated through the use of a 64bit
 *                      variable.
 *
 *      Fundamental functions which are visible outside the class
 *          ".updateAddressPointer" - Add a new Address pointer write request to the buffer
 *                                    (only function which puts a "WRITE" state into BME280 form)
 *          ".setupAddressAutoRead" - Add a new Address pointer read request to the buffer
 *                                    (only function which puts a "READ" state into BME280 form)
 *          ".lastAddresPointRqst"  - Retrieve the last requested/known state of the Address
 *                                    Pointer
 *          ".updateConfigReg"      - Construct the byte value to update Configuration Register as
 *                                    per new request
 *          ".deviceCalibrated"     - Returns "1" if the calibration data has been fully read from
 *                                    device.
 *
 *          ".deconstructData"      - Goes through the input buffer, and based upon the next
 *                                    BME280 form request decode the data
 *          ".calculateSensorReadings"
 *                                  - Calls the "compensateTemp"/"Pres"/"Humd" functions
 * 
 *  [#] BME280 Form System
 *      ~~~~~~~~~~~~~~~~~~
 *      The BME280 device utilises a "Address Pointer", which is updated as part of every write or
 *      read to the device, and is retained until either it is written again or a power cycle
 *      occurs.
 *      So as to manage this, the BME280 class has a internal "Form" system, where every
 *      read/write to the BME280 is captured along with the state of the Address Pointer. This is
 *      then used during the "deconstruct" phase.
 *      The deconstruct phase, will go through this list. Retrieving the state of the Address
 *      Pointer, and updating the class internal pointer state (which in theory should mimic the
 *      Address Pointer within the device). However if the request was for a WRITE, then no
 *      decoding occurs (as no data is read back); but Address Pointer update is retained. If
 *      request was a READ, then the data is decoded.
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef BME280_H_
#define BME280_H_

#include "FileIndex.h"
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

// Defines specific within this class
// Defined in the "BME280_defs.h" file
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class BME280 {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "BME280::" followed by the type.
 *************************************************************************************************/
public:
     enum class DevFlt : uint8_t {
         kNone                          = 0x00,
         kFault                         = 0x01,
         kSize_Request                  = 0x02,
         kUnrecognised_Address          = 0x03,
         kSynchronize_Error             = 0x04,

         kChipId_Wrong                  = 0x08,

         kNo_Communication              = 0xE0,
         kParent_Communication_Fault    = 0xF0,

         kInitialised                   = 0xFF
     };

    enum class ctrl_hum : uint8_t {
        kSkipped                        = 0x00,
        kHum_Oversampling_x1            = 0x01,
        kHum_Oversampling_x2            = 0x02,
        kHum_Oversampling_x4            = 0x03,
        kHum_Oversampling_x8            = 0x04,
        kHum_Oversampling_x16           = 0x05
    };

    enum class osrs_p : uint8_t {
        kSkipped                        = 0x00,
        kPrs_Oversampling_x1            = (0x01 << 5),
        kPrs_Oversampling_x2            = (0x02 << 5),
        kPrs_Oversampling_x4            = (0x03 << 5),
        kPrs_Oversampling_x8            = (0x04 << 5),
        kPrs_Oversampling_x16           = (0x05 << 5)
    };

    enum class osrs_t : uint8_t {
        kSkipped                        = 0x00,
        kTmp_Oversampling_x1            = (0x01 << 2),
        kTmp_Oversampling_x2            = (0x02 << 2),
        kTmp_Oversampling_x4            = (0x03 << 2),
        kTmp_Oversampling_x8            = (0x04 << 2),
        kTmp_Oversampling_x16           = (0x05 << 2)
    };

    enum class ctrl_mode : uint8_t {
        kSleep                          = 0x00,
        kForced1                        = 0x01,
        kForced2                        = 0x02,
        kNormal                         = 0x03
    };

    enum class t_sb : uint8_t {
        ktsmp_0_5ms                     = (0x00 << 5),
        ktsmp_62_5ms                    = (0x01 << 5),
        ktsmp_125_5ms                   = (0x02 << 5),
        ktsmp_250_0ms                   = (0x03 << 5),
        ktsmp_500_0ms                   = (0x04 << 5),
        ktsmp_1000_0ms                  = (0x05 << 5),
        ktsmp_10_0ms                    = (0x06 << 5),
        ktsmp_20_0ms                    = (0x07 << 5)
    };

    enum class cfig_filt : uint8_t {
        kOff                            = (0x00 << 2),
        kFilter_Coeff_2                 = (0x01 << 2),
        kFilter_Coeff_4                 = (0x02 << 2),
        kFilter_Coeff_8                 = (0x03 << 2),
        kFilter_Coeff_16                = (0x04 << 2),
        kFilter_Coeff_16_1              = (0x05 << 2),
        kFilter_Coeff_16_2              = (0x06 << 2),
        kFilter_Coeff_16_3              = (0x07 << 2)
    };

    typedef struct {
        uint8_t     AddrPoint;
        uint8_t     packetSize;
        enum Dir : uint8_t {kWrite, kRead} ReadWrite;
    } Form;

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
protected:
    GenBuffer<Form>      _address_buff_;    // Buffer for the state of the Address pointer,
                                            // needs to line up with any read backs (as this
                                            // indicates what has been read!)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t     _address_pointer_;  // Stores the current state of the Address Pointer

    ctrl_mode   _mode_;             // Internal copy of the device mode
    cfig_filt   _filt_;             // Internal copy of the device filter mode
    t_sb        _sample_rate_;      // Internal copy of the device sample rate

    ctrl_hum    _hum_oversamp_;     // Internal copy of the humity oversample
    osrs_p      _prs_oversamp_;     // Internal copy of the pressure oversample
    osrs_t      _tmp_oversamp_;     // Internal copy of the temperature oversample

    uint32_t    _calibration_points_;   // Count number of calibration points read so far
    struct _calib_data {            // Calibration data retrieved from the NVM of device
        uint16_t    dig_T1;
        int16_t     dig_T2;
        int16_t     dig_T3;

        uint16_t    dig_P1;
        int16_t     dig_P2;
        int16_t     dig_P3;
        int16_t     dig_P4;
        int16_t     dig_P5;
        int16_t     dig_P6;
        int16_t     dig_P7;
        int16_t     dig_P8;
        int16_t     dig_P9;

        uint8_t     dig_H1;
        int16_t     dig_H2;
        uint8_t     dig_H3;
        int16_t     dig_H4;
        int16_t     dig_H5;
        int8_t      dig_H6;
    }   _calibration_data_;


    uint32_t    _temp_raw_;         // Raw readings of Temperature, Humidity, Pressure
    uint32_t    _humd_raw_;         //
    uint32_t    _pres_raw_;         //
    int32_t     _t_fine_;           // Parameter used for most compensation calculations

public:
    DevFlt      flt;                // Fault of the device
    float      temperature;         // Calculated temperature in degC
    float      humidity;            // Calculated humidity in %RH
    float      pressure;            // Calculated pressure in Pa

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
protected:
    void popGenParam(void);

public:
    BME280(Form *FormArray, uint16_t FormSize);

    virtual ~BME280();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "BME280" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>  FUNDAMENTAL FUNCTION FOR CLASS TO WORK   <<<
             *   -----------
             *  These functions are the bases for use of this class. The "upper level" function
             *  (which are public), rely upon these functions to operate.
             *  Are protected, as the upper level functions will not need to use these.
             *************************************************************************************/
    // Function to go through the input array, and decode the specific data for all registers
    // note - is also called "Conversion" within the datasheet for BME280
    uint8_t parseRegisters(uint8_t *buff, uint8_t address);

    Form addressForm(uint8_t newAdd, Form::Dir Direction, uint8_t packetSize);

    void  compensateTemp(void);
    void  compensatePres(void);
    void  compensateHumd(void);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    BASIC  FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to setup and transfer packets of data to the selected
             *  BME280 device - Will wait for any hardware registers to be in correct state before
             *  progressing.
             *************************************************************************************/
    uint8_t updateAddressPointer(uint8_t *buff, uint8_t newval);
    // Generate write request to update Address Pointer in BME280 device
    uint8_t setupAddressAutoRead(uint8_t address, uint8_t numberregisters);
    uint8_t lastAddresPointRqst(void);              // Retrieve the last Address Pointer request

    uint8_t updateConfigReg(uint8_t *buff,
                            ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                            ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant);

    DevFlt deconstructData(uint8_t *buff, uint16_t size);
            // Go through input "GenBuffer" read data, and only deconstruct "size" entries.

    uint8_t deviceCalibrated(void);     // Will return "1" if all calibration data has been read
    void  calculateSensorReadings(void);// Function to be called to calculate the
                                        // Temperature/Pressure/Humidity readings
};

#endif /* BME280_H_ */
