/**************************************************************************************************
 * @file        Dynamixel.h
 * @author      Thomas
 * @version     V0.2
 * @date        06 Jul 2018
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * As this class is based upon the "UARTDevice" class, it is able to support multiple target
 * devices, and therefore the initial class call will vary depending upon the target:
 * The basic use of the class is the same for all target devices
 *      Call Class Dynamixel to initialise class
 *          For STM32F devices, providing the address of the UART handler - from cubeMX
 *          For RaspberryPi, provide the location of the serial interface, and the desired baudrate
 *
 *          Additional to this a size for the primary control array - "CommsBoard" need to be set,
 *          this is an additional argument to class construction.
 *              << NOTE >>
 *          The size specificed will be for the "CommsBoard" only, the intermediate arrays -
 *          "Transmit" and "Receive" from "UARTDevice" will be set to double this size.
 *
 *      Depending upon how the programmer wants to use the Dynamixel device, will change which
 *      functions are utilised.
 *      For functions to wait for new data, or data to be transmitted use:
 *          ".PoleTransmitPkg"      - Wait until can transmit data, then transmit as per protocol
 *          ".PoleReceievePkg"      - Wait for complete package to be read, then decode
 *
 *      If interrupts are to be used, then will first need to configure within the NVIC (which is
 *      not part of this class), then the following can be used:
 *          ".RequstTransmission"   - Request a transmission to occur
 *          ".IRQPreHandle"         - Periodic function call, to handle checking state of Dynamixel
 *                                    and either populate and enable transmission or decode
 *                                    complete package read.
 *          ".IRQHandle"            - Interrupt handler, to be called in Interrupt Routine
 *                                      -> How to do this is detailed in the source file for this
 *                                         function
 *      Static Function
 *          "Dynamixel::update_crc" - Function which can be called outside of class for calculating
 *                                    the required CRC for specified data
 *
 *      Functions which are not usable outside of the class:
 *          ".PkgSearch"            - Searches the contents of "CommsBoard" and determines if
 *                                    complete package has been read
 *          ".PkgCode"              - Will code request transmission data when state is equal to
 *                                    "Idle"
 *          ".PkgDecode"            - Will decode read data when Complete package has been found
 *          ".CleanBoard"           - Complete clear of "CommsBoard"
 *          ".QCleanBoard"          - Quick clean - just set pointers to zero
 *          ".UpdateWaterMark"      - Update watermark used to determine how much of "CommsBoard"
 *                                    has been used between cleans
 *
 *  If "__LiteImplement__" has been defined, then the class will not use "use" or "delete" to
 *  minimise the size impact. Therefore fully defined "GenBuffers" need to be provided to the
 *  constructors of the Dynamixel class.
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef DYNAMIXEL_DYNAMIXEL_H_
#define DYNAMIXEL_DYNAMIXEL_H_

#include <stdint.h>
#include "UARTDevice/UARTDevice.h"      // Allow use of UARTDevice class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// Add includes specific to the STM32Fxx devices

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Add includes specific to the Raspberry Pi

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
#define     Dynm_Instr_Ping     0x01    // Instruction code for Pinging to device
#define     Dynm_Instr_Read     0x02    // Instruction code for Reading from a device
#define     Dynm_Instr_Wrte     0x03    // Instruction code for Writing to a device
#define     Dynm_Instr_RegW     0x04    // Instruction code for Register write (won't be written
                                        // till "Action" instruction is sent)
#define     Dynm_Instr_Acti     0x05    // Instruction code for Action
#define     Dynm_Instr_FctR     0x06    // Instruction code for returning factory conditions
#define     Dynm_Instr_Rebt     0x08    // Instruction code for Rebooting device

#define     Dynm_Instr_Stts     0x55    // Instruction code for Return status from device

#define     Dynm_Instr_SyRd     0x82    // Instruction code for multiple device read - same address
#define     Dynm_Instr_SyWt     0x83    // Instruction code for multiple device write - as above

#define     Dynm_Instr_BkRd     0x92    // Instruction code for multiple device read - dif address
#define     Dynm_Instr_BkWt     0x93    // Instruction code for multiple device write - as above

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

// Types used within this class
typedef enum {
    Dynm_Idle       = 0,
    Dynm_Speaking   = 1,
    Dynm_Listening  = 2
} _DynmState;

typedef enum {
    Dynm_Nothing    = 0,
    Dynm_Header     = 1,
    Dynm_Complete   = 2
} _DynmSearch;

class Dynamixel : public UARTDevice {
    private:
        uint8_t         *CommsBoard;                    // Pointer to array to store single wire
                                                        // UART communication
        //uint8_t         CommsBoard[128];
        uint16_t        Length;                         // Size of array
        uint16_t        curpoint;                       // Pointer to current position
        uint16_t        maxpoint;                       // Pointer to maximum size of data
        uint16_t        watermark;                      // Pointer to the maximum size of data
                                                        // between BoardCleans

    public:     // private
        _DynmState      State;          // Configure the status of the device
        _DynmSearch     SrcState;       // States to store listening mode

// Device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        Dynamixel(UART_HandleTypeDef *UART_Handle, uint8_t *CommsBoardLoc, uint16_t size,
                  GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray);
        // Setup the Dynamixel interface by providing the UART handle provided by cubeMX, along
        // with fully defined arrays and "GenBuffers" for internal data management
#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        Dynamixel::Dynamixel(UART_HandleTypeDef *UART_Handle, uint16_t size);
        // Setup the Dynamixel interface by providing the UART handle provided by cubeMX, along
        // the size for the internal buffer arrays

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        Dynamixel(const char *deviceloc, int baud, uint16_t size,
                  GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray)
        // Setup the Dynamixel interface by device location, and desired baudrate. Also requires
        // fully defined arrays and "GenBuffers" for internal data management

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        Dynamixel(const char *deviceloc, int baud, uint16_t size);
        // Setup the Dynamixel interface by device location, and desired baudrate. Also requires
        // fully defined arrays and "GenBuffers" for internal data management
#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#else
//==================================================================================================
    public:
        UARTDevice();

#endif

private:
    _DynmSearch PkgSearch(void);        // Function will search through data on "CommsBoard" and
                                        // see if a complete package is present
    void PkgCode(void);                 // Will look into "Transmit" buffer, and if state of
                                        // Dynamixel is correct, will build package content
    uint8_t PkgDecode(uint8_t *ID, uint8_t *Instr, uint16_t *size);
    // Once complete package has been read, this will decode the contents and provide the
    // Packet ID, Instruction and size - parameter data will need to be read from "Receive" buffer

    void CleanBoard(void);              // Clean the contents of "CommsBoard" upto watermark
    void QCleanBoard(void);             // Quick clean, by setting pointers to zero
    void UpdateWaterMark(void);         // Update watermark based upon bigger of itself and
                                        // maxpoint

public:
    static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    // Calculate the CRC for defined input

    // 1> Poling functions to transmit package(s) and receive package(s)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    _UARTDevFlt PoleTransmitPkg(uint8_t ID, uint8_t Instr, uint8_t *param, uint16_t size);
        // Function to transmit data in the Dynamixel protocol
    uint8_t PoleReceievePkg(uint8_t *ID, uint8_t *Instr, uint16_t *size);
    // Function to receive data in the Dynamixel protocol

    // 2> Interrupt based versions of transmitting and receiving package(s)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    _UARTDevFlt RequstTransmission(uint8_t ID, uint8_t Instr, uint8_t *param, uint16_t size);
        // Request a transmission to data to occur - will only occur when state is equal to "Idle"
        // via periodic call "IRQPreHandle"
    uint8_t IRQPreHandle(uint8_t *ID, uint8_t *Instr, uint16_t *size);
        // Function used to populate "CommsBoard" and enable transmission interrupt. Along with
        // Decoding "CommsBoard" when complete package has been read

    void IRQHandle(void);               // Interrupt handler

    virtual ~Dynamixel();
};

#endif /* DYNAMIXEL_DYNAMIXEL_H_ */
