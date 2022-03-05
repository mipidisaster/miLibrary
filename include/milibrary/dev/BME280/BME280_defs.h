/**************************************************************************************************
 * @file        BME280_defs.h
 * @author      Thomas
 * @brief       Header file for the BMR280 series of temperature sensors (Definitions only)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Script includes all the defines needed for interfacing with the BME280 device - either SPI or
 * I2C.
 *************************************************************************************************/
#ifndef BME280_defs_H_
#define BME280_defs_H_

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
// None

//=================================================================================================

// Defines specific within this class
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// BME280 Registers
// ~~~~~~~~~~~~~~~~
#define BME280_ID               0xD0                // Chip ID register
#define BME280_RESET            0xE0                // Reset Register

#define BME280_CTRL_HUM         0xF2                // Humidity control Register
#define BME280_CTRL_HUM_MASK    0x07                // Mask for the bits for Humidity control

#define BME280_STTS             0xF3                // Status of device bits
#define BME280_STTS_CONV        0x04                // Bit automatically set to "1" whilst
                                                    // conversion is underway
#define BME280_STTS_IM_UPDATE   0x01                // Bit automatically set to "1" whilst NVM data
                                                    // is being copied to registers

#define BME280_CTRL_MEAS        0xF4                // Control Measurement Register
#define BME280_CTRL_P_MASK      0xE0                // Mask to get the Pressure Sampling rate
#define BME280_CTRL_T_MASK      0x1C                // Mask to get the Temperature Sampling rate
#define BME280_CTRL_MODE_MASK   0x03                // Mask to get the Mode of the device

#define BME280_CFIG             0xF5                // Configuration of the device
#define BME280_CFIG_TSMP_MASK   0xE0                // Mask to get the standby time in normal mode
#define BME280_CFIG_FILT_MASK   0x1C                // Mask to get the IIR filter
#define BME280_CFIG_SPI_MASK    0x01                // Mask to setup 3wire SPI

#define BME280_PRES             0xF7                // Pressure read register
#define BME280_TEMP             0xFA                // Temperature read register
#define BME280_HUMD             0xFD                // Humidity read register

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Calibration Registers
// ~~~~~~~~~~~~~~~~~~~~~
#define BME280_T1_LSB           0x88
#define BME280_T2_LSB           0x8A
#define BME280_T3_LSB           0x8C
#define BME280_TEMP_CAL_COUNT   6       // Number of bytes to be read for Temperature Calibration
#define BME280_TEMP_CAL_MASK    0x00000007  //

// 18bits

#define BME280_P1_LSB           0x8E
#define BME280_P2_LSB           0x90
#define BME280_P3_LSB           0x92
#define BME280_P4_LSB           0x94
#define BME280_P5_LSB           0x96
#define BME280_P6_LSB           0x98
#define BME280_P7_LSB           0x9A
#define BME280_P8_LSB           0x9C
#define BME280_P9_LSB           0x9E
#define BME280_PRES_CAL_COUNT   18      // Number of bytes to be read for Pressure    Calibration
#define BME280_PRES_CAL_MASK    0x00000FF8  //

#define BME280_H1_8BIT          0xA1
#define BME280_H2_LSB           0xE1
#define BME280_H3_8BIT          0xE3
#define BME280_H45_3bytes       0xE4
#define BME280_H6_8BIT          0xE7
#define BME280_HUMD_CAL_MASK    0x0003F000    //
/* Not including a count for Humidty, as it broken up into multiple areas, specific logic will
 * be created to support this
 */
//#define BME280_HUMD_CAL_COUNT   18      // Number of bytes to be read for Humidty     Calibration

#define __BME280_DATA_REGISTERS(_REG_) \
    ( (_REG_ >= BME280_T1_LSB)   && (_REG_ <= BME280_P9_LSB) )  || \
    (  _REG_ == BME280_H1_8BIT)                                 || \
    ( (_REG_ >= BME280_H2_LSB)   && (_REG_ <= BME280_H6_8BIT) ) || \
    ( (_REG_ >= BME280_CTRL_HUM) && (_REG_ <= BME280_CFIG) )    || \
    ( (_REG_ >= BME280_PRES)     && (_REG_ <= BME280_HUMD) )

// Section 3.5.1 of BME280 datasheet
#define BME280_RECOMMENDED_SETTING_WEATHER          \
            BME280::ctrl_hum::kHum_Oversampling_x1, \
            BME280::osrs_p::kPrs_Oversampling_x1,   \
            BME280::osrs_t::kTmp_Oversampling_x1,   \
            BME280::ctrl_mode::kForced1,            \
            BME280::t_sb::ktsmp_0_5ms,              \
            BME280::cfig_filt::kOff

// Section 3.5.2 of BME280 datasheet
#define BME280_RECOMMENDED_SETTING_HUMIDTY          \
            BME280::ctrl_hum::kHum_Oversampling_x1, \
            BME280::osrs_p::kSkipped,               \
            BME280::osrs_t::kTmp_Oversampling_x1,   \
            BME280::ctrl_mode::kForced1,            \
            BME280::t_sb::ktsmp_0_5ms,              \
            BME280::cfig_filt::kOff

// Section 3.5.3 of BME280 datasheet
#define BME280_RECOMMENDED_SETTING_INDOOR           \
            BME280::ctrl_hum::kHum_Oversampling_x1, \
            BME280::osrs_p::kPrs_Oversampling_x16,  \
            BME280::osrs_t::kTmp_Oversampling_x2,   \
            BME280::ctrl_mode::kNormal,             \
            BME280::t_sb::ktsmp_0_5ms,              \
            BME280::cfig_filt::kFilter_Coeff_16

// Section 3.5.4 of BME280 datasheet
#define BME280_RECOMMENDED_SETTING_GAMING           \
            BME280::ctrl_hum::kSkipped,             \
            BME280::osrs_p::kPrs_Oversampling_x4,   \
            BME280::osrs_t::kTmp_Oversampling_x1,   \
            BME280::ctrl_mode::kNormal,             \
            BME280::t_sb::ktsmp_0_5ms,              \
            BME280::cfig_filt::kFilter_Coeff_16

// I2C Address for the BME280
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
#define BME280_I2CADDRSS_GND    0x0076              // I2C Address for device SDO = GND
#define BME280_I2CADDRSS_VDD    0x0077              // I2C Address for device SDO = Vcc

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

// Types used within this class
// None

#endif /* BME280_defs_H_ */
