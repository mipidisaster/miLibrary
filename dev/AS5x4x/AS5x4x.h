/**************************************************************************************************
 * @file        AS5x4x.h
 * @author      Thomas
 * @version     V0.1
 * @date        20 Oct 2018
 * @brief       Header file for the AMS Angular Position device (AS5x4x)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communications between the target embedded device and the
 * selected AS5x4x device.
 *      Call Class AS5x4x to initialise class, detail the attached device.
 *      Currently only supports:
 *          AS5047D
 *          AS5048A
 *
 *      If in "LiteImplementation" mode, then pointers to the write and read buffers need to be
 *      provided. Otherwise they will be generated internally to the class.
 *
 *      Functions used internal to the class:
 *          ".WriteDataPacket"      - Put packet requested into the write buffer
 *          ".ReadDataPacket"       - Read the packet from device from read buffer
 *
 *          ".deconstructAS5048A"   - Deconstruct the AS5048A data from device
 *          ".deconstructAS5047D"   - Deconstruct the AS5047D data from device
 *
 *      Functions to be used to request/read data from AS5x4x device
 *          ".SPIWriteChain"        - SPI Write Daisy Chain for AS5x4x device
 *          ".SPIReadChain"         - SPI Read Daisy Chain for AS5x4x device
 *
 *          ".DirectSPITransmit"    - Directly transmit data via SPI to a single AS5x4x device
 *
 *      Functions to requested data from the AS5x4x device:
 *          constructNOP, constructCEF, constructAGC, constructMag, constructAng
 *          constructALL
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef AS5X4X_H_
#define AS5X4X_H_

#include "FileIndex.h"
#include <stdint.h>

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_SPIDe__HD               // Allow use of the SPI Device class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// None

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// Generic Defines for AS5x4x devices
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > SPI Communication Command Packages
#define AS5x4x_PARITY   0x8000          // Position for EVEN Parity bit
#define AS5x4x_ReadMask 0x4000          // Position for the Write/Read bit (0 / 1 respectively)
#define AS5x4x_DataMask 0x3FFF          // Mask to get the address sent only
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > Error Register
#define AS5x4x_ErrorMask                0x0007      // Mask for getting device errors
#define AS5x4x_FrameError               0x0001      // Mask for Framing error
#define AS5x4x_CmdIvError               0x0002      // Mask for Command Invalid
#define AS5x4x_ParitError               0x0004      // Mask for Parity error

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > AGC Register Mask
#define AS5x4x_AGVValueMask             0x00FF      // Mask to get the Automatic Gain Control value
#define AS5x4x_DiagnFlags               0x0F00      // Mask to get the Diagnostic flags
#define AS5x4x_OCFMask                  0x0001      // Mask to get the Offset Compensation
                                                    // Algorithm flag
#define AS5x4x_COFMask                  0x0002      // Mask to get the CORDIC Overflow flag
#define AS5x4x_CompLow                  0x0004      // Mask to get the Comp Low flag
#define AS5x4x_CompHigh                 0x0008      // Mask to get the Comp High flag

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AS5x47
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AS5047_NOP      0x0000          // Address for No operation message
#define AS5047_ERRFL    0x0001          // Address for the Error Register
#define AS5047_PROG     0x0003          // Address for the Programming Register
#define AS5047_DIAAGC   0x3FFC          // Address for the Diagnostic and AGC Register
#define AS5047_MAG      0x3FFD          // Address for the CORDIC magnitude
#define AS5047_ANGLE    0x3FFF          // Address for the Measured angle with dynamic angle error
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AS5x48
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AS5048_NOP      0x0000          // Address for No operation message
#define AS5048_ERRFL    0x0001          // Address for the Error Register
#define AS5048_PROG     0x0003          // Address for the Programming Register
#define AS5048_DIAAGC   0x3FFD          // Address for the Diagnostic and AGC Register
#define AS5048_MAG      0x3FFE          // Address for the CORDIC magnitude
#define AS5048_ANGLE    0x3FFF          // Address for the Measured angle
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

#ifndef PI
#define PI                 3.14159265358979f
#endif

// Types used within this class
typedef enum {
    AS5x4x_NoFault      = 0,
    AS5x4x_Frame        = 1,
    AS5x4x_CmdInv       = 2,
    AS5x4x_Parity       = 3,

    AS5x4x_Initialised = -1
} _AS5x4xFlt;

typedef enum {
    AS5048A         = 0,
    AS5047D         = 1
} _AS5x4xDev;

class AS5x4x {
protected:
    GenBuffer<uint16_t> *rdBuff;        // Read buffer for data transfer
    GenBuffer<uint16_t> *wtBuff;        // Write buffer for data transfer

public:
    float       Angle;                  // Calculated real angle from device (radians)
    uint16_t    AngularSteps;           // Read number of Angular steps (0 .. 16384)
    uint16_t    Mag;                    // Magnitude of Magnetic flux from CORDIC
                                        // CORDIC = Coordinate Rotation Digital Computer
    uint16_t    AGC;                    // Automatic Gain Control value
    uint16_t    Diagnostic;             // Diagnostic flags
    _AS5x4xFlt  Flt;                    // Fault status of the device

    _AS5x4xDev  Device;                 // Store the device type

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        AS5x4x(_AS5x4xDev Device, GenBuffer<uint16_t> *wtBuff, GenBuffer<uint16_t> *rdBuff);
        // Setup the AS5x4x device, by providing the device type, as well as the "GenBuffer"
        // to be used as the internal write and read buffer.

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        AS5x4x(_AS5x4xDev Device, uint32_t Buffersize)
    // Setup the AS5x4x device, by providing the device type and the size of the internal buffers

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // 0> Basic functions for organising data
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    static uint8_t EvenParityCheck(uint16_t packet);    // Determine if the data packet is even
                                                        // parity
    void WriteDataPacket(uint16_t PacketData);          // Check data for parity, and put into
                                                        // Buffer
    void ReadDataPacket(void);                          // Read the contents of the 16bit buffer

    // 1> SPI peripheral functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    static uint16_t SPIWriteChain(AS5x4x *device, uint16_t numchain, uint8_t *wtdata);
    static uint8_t SPIReadChain(AS5x4x *device, uint16_t numchain, uint8_t *rddata, uint16_t size);

    void DirectSPITransmit(SPIDevice *Interface, GPIO *CS);

    // 2> Functions to request and understand data from the AS5x4x devices
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void constructNOP(void);            // Function to request a NOP transmission
    void constructCEF(void);            // Function to request a CEF transmission
    void constructAGC(void);            // Function to request a AGC transmission
    void constructMag(void);            // Function to request a Mag transmission
    void constructAng(void);            // Function to request a Ang transmission

    void constructALL(void);            // Function to request full update of device

    void deconstructAS5048A(uint16_t Address, uint16_t packetdata); // Deconstruct AS5048A data
    void deconstructAS5047D(uint16_t Address, uint16_t packetdata); // Deconstruct AS5047D data

    virtual ~AS5x4x();
};

#endif /* AS5X4X_H_ */
