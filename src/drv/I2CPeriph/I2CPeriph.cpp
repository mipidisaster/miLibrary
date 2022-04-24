/**************************************************************************************************
 * @file        I2CPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic I2C Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_I2CPe__HD               // Header for I2C

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL I2C library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL I2C library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#include <stdio.h>                      // Standard I/O - including the error file
#include <stdarg.h>                     // Allows functions to accept an indefinite number of
                                        // arguments
#include <stdlib.h>                     // Needed for 'exit'

#include <fcntl.h>                      // Needed for I2C port
#include <unistd.h>                     //
#include <sys/ioctl.h>                  // Control of I/O devices
#include <linux/i2c-dev.h>              // ioctl I2C specific parameters
// See  https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface
//      https://kernel.org/doc/Documentation/i2c/dev-interface
//          https://linux.die.net/

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
#include <stdio.h>                      // Standard I/O - including the error file
#include <stdarg.h>                     // Allows functions to accept an indefinite number of
                                        // arguments
#include <stdlib.h>                     // Needed for 'exit'

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================

void I2CPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the I2CPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    flt           = DevFlt::kInitialised;   // Initialise the fault to "initialised"
    comm_state    = CommLock::kFree;        // Indicate bus is free

    _cur_count_    = 0;                     // Initialise the current packet size count
    _cur_form_     = { 0 };                 // Initialise the form to a blank entry

    _cur_reqst_    = Request::kNothing;     // Initialise the current request state to 0
}

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
I2CPeriph::I2CPeriph(I2C_HandleTypeDef *I2C_Handle, Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * Creates a I2C class specific for the STM32 device.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired I2C
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    popGenParam();                    // Populate generic class parameters

    _i2c_handle_  = I2C_Handle;       // Link input I2C handler to class.

    _form_queue_.create(FormArray, FormSize);
}
#else   // Raspberry Pi or Default build configuration
//=================================================================================================
I2CPeriph::I2CPeriph(const char *deviceloc, uint16_t FormSize) {
/**************************************************************************************************
 * Create a I2CPeriph class specific for RaspberryPi, to simplify the RaspberryPi version (and as
 * size is less of a constraint) function will create arrays for the internal Form buffers.
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    popGenParam();              // Populate generic class parameters

    _pseudo_interrupt_  = 0x00;         // pseudo interrupt register used to control the I2C
                                        // interrupt for Raspberry Pi
    _device_loc_        = deviceloc;    // Capture the folder location of I2C device

    _form_queue_.create(new Form[FormSize], FormSize);

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    _i2c_handle_        = open(_device_loc_, O_RDWR);   // Open file to the I2C Device

    if (_i2c_handle_ < 0)
        errorMessage("Unable to open I2C device: %s", _device_loc_);

#else
    _i2c_handle_                = 0;
    _return_message_            = {"No I2C Hardware Attached"};
    _message_size_              = 24;
    _return_message_pointer_    = 0;

#endif
}

I2CPeriph::I2CPeriph(const char *deviceloc, Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * Create a I2CPeriph class specific for RaspberryPi
 *  This version requires pointers to the Write/Read I2C Form system and sizes
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    popGenParam();              // Populate generic class parameters

    _pseudo_interrupt_  = 0x00;         // pseudo interrupt register used to control the I2C
                                        // interrupt for Raspberry Pi
    _device_loc_        = deviceloc;    // Capture the folder location of I2C device

    _form_queue_.create(FormArray, FormSize);

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    _i2c_handle_        = open(_device_loc_, O_RDWR);   // Open file to the I2C Device

    if (_i2c_handle_ < 0)
        errorMessage("Unable to open I2C device: %s", _device_loc_);

#else
    _i2c_handle_                = 0;
    _return_message_            = {"No I2C Hardware Attached"};
    _message_size_              = 24;
    _return_message_pointer_    = 0;

#endif
}

void I2CPeriph::errorMessage(const char *message, ...) {
/**************************************************************************************************
 * Set message for failure set, and include the specific error code.
 *************************************************************************************************/
    char buffer[1024];
    va_list args;

    va_start(args, message);                    // Capture the extra arguments in function input
    vsnprintf(buffer, 1024, message, args);     //
    perror(buffer);                             // Add the error code/message to string
    va_end(args);

    exit (EXIT_FAILURE);                        // Close down program
}

void I2CPeriph::pseudoRegisterSet(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function set the desired 'entry' of '_pseudo_interrupt_'
 *************************************************************************************************/
    _pseudo_interrupt_  |= entry;
}

void I2CPeriph::pseudoRegisterClear(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function clear the desired 'entry' of '_pseudo_interrupt_'
 *************************************************************************************************/
    _pseudo_interrupt_  &= ~(entry);
}

uint8_t I2CPeriph::pseudoStatusChk(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function see if the desired 'entry' of '_pseudo_interrupt_' is set
 *************************************************************************************************/
    return ( _pseudo_interrupt_ & entry );
}

#endif
uint8_t I2CPeriph::readDR(void) {
/**************************************************************************************************
 * Read from the I2C hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    return ((uint8_t) _i2c_handle_->Instance->RXDR);    // Read from the RX Data Register

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library or Default configuration
// doesn't need this . Function will not be called by upper level functions
    return 0;

#endif
}

void I2CPeriph::writeDR(uint8_t data) {
/**************************************************************************************************
 * Write to the I2C hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    _i2c_handle_->Instance->TXDR = data;                // Put data onto the TX Data Register

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library or Default configuration
// doesn't need this . Function will not be called by upper level functions
// Do nothing!

#endif
}

uint8_t I2CPeriph::transmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
//  The STM32L device has two bits which can be used to determine if the Transmit buffer is empty,
//  they are TXE = Transmit Buffer empty bit, and the TXIE = Transmit Buffer status
//     TXIE is used to trigger the interrupt
//  Both are used within this function call.
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_TXE)  != 0 ) || \
         (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_TXIS) != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t I2CPeriph::transmitComptChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit communication (if completed, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// I2C communication for STM32L has two "Transmission Complete" flags, one is generic, the other
// is used for Complete transmission for I2C Reload
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_TC)   != 0 ) || \
         (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_TCR)  != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t I2CPeriph::receiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_RXNE) != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t I2CPeriph::busNACKChk(void) {
/**************************************************************************************************
 * Check to see if peripheral has NOT acknowledge the packet (if NACK, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_AF)   != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (packet has been accepted),
                                        // such that any downstream will continue as if entry is
                                        // set
#endif
}

void I2CPeriph::clearNACK(void) {
/**************************************************************************************************
 * Clear the NACK bit in the status register
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_I2C_CLEAR_FLAG(_i2c_handle_, I2C_FLAG_AF);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
}

uint8_t I2CPeriph::busStopChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus has been "STOPPED" (if bus is STOP, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_STOPF) != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output (bus has been STOPPED), such
                                        // that any downstream will continue as if entry is set
#endif
}

void I2CPeriph::clearStop(void) {
/**************************************************************************************************
 * Clear the Bus STOP bit status
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_I2C_CLEAR_FLAG(_i2c_handle_, I2C_FLAG_STOPF);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
}

uint8_t I2CPeriph::busBusyChk(void) {
/**************************************************************************************************
 * Check to see if the I2C bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_BUSY) != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (bus is not busy),such that
                                        // any downstream will continue as if entry is set

#endif
}

uint8_t I2CPeriph::busErroChk(void) {
/**************************************************************************************************
 * Check to see if there has been an incorrect START/STOP bit sent (if fault, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( (__HAL_I2C_GET_FLAG(_i2c_handle_, I2C_FLAG_BERR) != 0 ) )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (no bus error),such that
                                        // any downstream will continue as if entry is set

#endif
}

void I2CPeriph::clearBusEr(void) {
/**************************************************************************************************
 * Clear the Bus error status bit
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_I2C_CLEAR_FLAG(_i2c_handle_, I2C_FLAG_BERR);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
}

uint8_t I2CPeriph::transmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_TXI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(ktransit_data_register_empty) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t I2CPeriph::transmitComptITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Complete Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_TCI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(ktransmit_complete) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t I2CPeriph::receiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_RXI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(kread_data_register_not_empty) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t I2CPeriph::busNACKITChk(void) {
/**************************************************************************************************
 * Check to see whether the Negative Acknowledge interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_NACKI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(kbus_NACK) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t I2CPeriph::busStopITChk(void) {
/**************************************************************************************************
 * Check to see whether the STOP bus interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_STOPI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(kbus_stop) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t I2CPeriph::busErrorITChk(void) {
/**************************************************************************************************
 * Check to see whether the Bus Error interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (__HAL_I2C_GET_IT_SOURCE(_i2c_handle_, I2C_IT_ERRI) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(kbus_error) != 0 )
        return (1);
    else
        return (0);

#endif
}

void I2CPeriph::requestTransfer(uint16_t devAddress, uint8_t size, CommMode mode, Request reqst) {
/**************************************************************************************************
 * Function will configure the device hardware/internal class parameters to setup a I2C
 * communication between this device (local) and the selected device (external). This communicate
 * can be either read/write, however the maximum number of packets which can be transmitted is
 * limited to 255 (1 byte). If more is required then this function will need to be called multiple
 * times with divisions of 255 to complete the communication.
 * The devAddress that is provided needs to be provided as a 10bit or 7bit address; function will
 * understand this, and setup communication correctly.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// Not populated yet

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// For the STM32Lx device the configuration of I2C for data transmission/reception is straight
// forward, all that is required is to populate the CR2 hardware register with Device Address
// Read/Write status, and populate the number of packets to send

    // Clear the CR2 register of generic conditions:
    //          Slave Address       (SADD)
    //          Bytes to transmit   (NBYTES) (limited to 255)
    //          Reload/AutoEnd      (RELOAD/AUTOEND)
    //          START/STOP          (START/STOP)
    _i2c_handle_->Instance->CR2 &= ~((I2C_CR2_SADD   | \
                                      I2C_CR2_NBYTES | \
                                      I2C_CR2_RELOAD | I2C_CR2_AUTOEND | \
                                      I2C_CR2_START  | I2C_CR2_STOP));
    // If the request is anything other than NOTHING, then clear the Read/~Write bit
    if (reqst != Request::kNothing)
        _i2c_handle_->Instance->CR2 &= ~(I2C_CR2_RD_WRN);
    // Now the register has been cleared, so can now populate with the parameters for the new
    // transfer
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup Slave Address:
    devAddress <<= 1;           // Input 'devAddress' will be the 'raw' address. So will need to
                                // shift the address up by 1 (to make 11bit/8bit)
    _i2c_handle_->Instance->CR2 |= ((uint32_t)devAddress & I2C_CR2_SADD);

    // Setup the size of data to be transmitted:
    _i2c_handle_->Instance->CR2 |= (((uint32_t)size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES);

    _cur_count_   = size;       // Capture the number of packets to transmit

    // Setup the communication mode:
    if      (mode == CommMode::kAutoEnd)                // If mode is "AutoEnd"
        _i2c_handle_->Instance->CR2 |= (uint32_t)(I2C_AUTOEND_MODE);
        // Set the AutoEnd bit in register
    else if (mode == CommMode::kReload)                 // If mode is "Reload"
        _i2c_handle_->Instance->CR2 |= (uint32_t)(I2C_RELOAD_MODE);
        // Set the RELOAD bit in register
    // The other mode "I2C_SoftEnd", is set by clearing both of these bits. Which is done during
    // the register clear part.

    // Setup the Request mode
    if      (reqst == Request::kStart_Write) {          // If request is for START_WRITE
        _i2c_handle_->Instance->CR2 |= (uint32_t)(I2C_CR2_START);
        // just set START bit (write is done, by clearing the read bit)
        _cur_reqst_ = reqst;    // Bring across the request mode
    }
    else if (reqst == Request::kStart_Read) {           // If request is for START_READ
        _i2c_handle_->Instance->CR2 |= (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN);
        // Set the START bit, along with the Read bit
        _cur_reqst_ = reqst;     // Bring across the request mode
    }
    else if (reqst == Request::kStop)                   // If request is for STOP
        _i2c_handle_->Instance->CR2 |= (uint32_t)(I2C_CR2_STOP);
        // Set the STOP bit

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// Configure the device in communication with external device, with address 'devAddress'
    if (ioctl (_i2c_handle_,  I2C_SLAVE, devAddress)            < 0)
        errorMessage("Unable to select I2C device: %X", devAddress);

    _cur_reqst_ = reqst;    // Bring across the request mode

#else
//=================================================================================================
// As not external hardware do not need to configure the hardware to a specific external device's
// address

    _cur_reqst_ = reqst;    // Bring across the request mode

#endif

}

I2CPeriph::Form I2CPeriph::genericForm(uint16_t devAddress, uint16_t size, CommMode mode,
                                       Request reqst,
                                       volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Generate a I2CForm request, based upon the generic information provided as input.
 *************************************************************************************************/
    Form request_form = { 0 };                  // Generate the "Form" variable to provide as 
                                                // output

    request_form.devAddress      = devAddress;  // Populate form with input data
    request_form.size            = size;        //
    request_form.Reqst           = reqst;       //
    request_form.Mode            = mode;        //

    // Indications used for source functionality to get status of requested communication
    request_form.Flt             = fltReturn;   // Populate return fault flag
    request_form.Cmplt           = cmpFlag;     // Populate complete communication indication

    return (request_form);
}

void I2CPeriph::formW8bitArray(I2CPeriph::Form *RequestForm, uint8_t *pData) {
/**************************************************************************************************
 * Link input 8bit array pointer to the provided I2C Request Form.
 *************************************************************************************************/
    RequestForm->Buff       = pData;        // Pass data pointer to I2CForm
        // Indicate that data type is 8bit array.
}

void I2CPeriph::specificRequest(uint16_t devAddress, uint16_t size, uint8_t *pData,
                        CommMode mode, Request reqst,
                        volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function used to populate the internal I2C Form stack, with the input requested communication.
 * Data comes from an array (pointer - pData)
 *************************************************************************************************/
    Form request_form = genericForm(devAddress, size, mode, reqst, fltReturn, cmpFlag);

    formW8bitArray(&request_form, pData);


    _form_queue_.inputWrite(request_form);      // Put request onto I2C Form Queue
}

uint8_t I2CPeriph::getFormWriteData(Form *RequestForm) {
/**************************************************************************************************
 * Retrieve the next data point to write to external device from the selected I2C Request form
 *************************************************************************************************/
    uint8_t temp_val = 0;       // Temporary variable to store data value

    temp_val = *(RequestForm->Buff);            // Retrieve data from array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer

    return (temp_val);      // Return value outside of function
}

void I2CPeriph::putFormReadData(Form *RequestForm, uint8_t readdata) {
/**************************************************************************************************
 * Data read from the I2C external device is copied into the requested source location, as per
 * the I2C Request form
 *************************************************************************************************/
    *(RequestForm->Buff)    = readdata;         // Put data into array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer
}

I2CPeriph::DevFlt I2CPeriph::poleMasterTransmit(uint16_t devAddress, uint8_t *pdata,
                                                uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * send the requested amount of data to be written to the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    comm_state = CommLock::kCommunicating;      // Indicate bus is communicating

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    //Enable();                     // Ensure that the device has been enabled

    while(busBusyChk() == 1) {}   // Wait until the bus is no longer busy

    requestTransfer(devAddress, size, CommMode::kAutoEnd, Request::kStart_Write);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    while(size != 0) {
        while(transmitEmptyChk() == 0) {    // Whilst checking for Transmit Empty
            if (busNACKChk() == 1) {        // If there is a NACK
                clearNACK();                // Clear the NACK bit
                return (flt = DevFlt::kNACK);       // Fault status of "I2C_NACK"
            }

            if (busErroChk() == 1) {        // If there has been a bus error
                clearBusEr();               // Clear the Bus error bit
                return (flt = DevFlt::kBus_Error);  // Fault status of "I2C_BUS_ERROR"
            }
        }
        writeDR(*pdata);                    // Put data onto the TXDR for transmission
        pdata++;                            // Increment array
        size--;                             // Decrease size
    }

    while(busStopChk() == 0) {              // Wait on the STOP to be set
        if (busNACKChk() == 1) {            // If there is a NACK
            clearNACK();                    // Clear the NACK bit
            return (flt = DevFlt::kNACK);           // Fault status of "I2C_NACK"
        }

        if (busErroChk() == 1) {            // If there has been a bus error
            clearBusEr();                   // Clear the Bus error bit
            return (flt = DevFlt::kBus_Error);      // Fault status of "I2C_BUS_ERROR"
        }
    }

    clearStop();                    // Clear the STOP bit

    //Disable();                    // Ensure that the device has been disable

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    //Enable();                     // Ensure that the device has been enabled

    while(busBusyChk() == 1) {}   // Wait until the bus is no longer busy

    requestTransfer(devAddress, size, CommMode::kAutoEnd, Request::kStart_Write);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    if (write(_i2c_handle_, pdata, size) != size) {     // If the requested number of bytes is not
                                                        // transmitted
        return (flt = DevFlt::kBus_Error);              // Set fault
    }

    while(busStopChk() == 0) {              // Wait on the STOP to be set
        if (busNACKChk() == 1) {            // If there is a NACK
            clearNACK();                    // Clear the NACK bit
            return (flt = DevFlt::kNACK);           // Fault status of "I2C_NACK"
        }

        if (busErroChk() == 1) {            // If there has been a bus error
            clearBusEr();                   // Clear the Bus error bit
            return (flt = DevFlt::kBus_Error);      // Fault status of "I2C_BUS_ERROR"
        }
    }

    clearStop();                    // Clear the STOP bit

    //Disable();                    // Ensure that the device has been disable


#else
//=================================================================================================
// Device not supported

#endif

    // Indicate that the bus is free
    comm_state = CommLock::kFree;       // Indicate bus is free

    return (flt = DevFlt::kNone);       // No fault by this point, so return no fault
}

I2CPeriph::DevFlt I2CPeriph::poleMasterReceive(uint16_t devAddress, uint8_t *pdata,
                                               uint8_t size) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * read back the request amount of data from the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    comm_state = CommLock::kCommunicating;      // Indicate bus is communicating

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    //Enable();                     // Ensure that the device has been enabled

    while(busBusyChk() == 1) {}   // Wait until the bus is no longer busy

    requestTransfer(devAddress, size, CommMode::kAutoEnd, Request::kStart_Read);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    while(size != 0) {
        while(receiveToReadChk() == 0) {    // Whilst checking for Receive buffer is full
            if (busNACKChk() == 1) {        // If there is a NACK
                clearNACK();                // Clear the NACK bit
                return (flt = DevFlt::kNACK);       // Fault status of "I2C_NACK"
            }

            if (busErroChk() == 1) {        // If there has been a bus error
                clearBusEr();               // Clear the Bus error bit
                return (flt = DevFlt::kBus_Error);  // Fault status of "I2C_BUS_ERROR"
            }
        }
        *pdata = readDR();                  // Read the data from RXDR into array
        pdata++;                            // Increment array
        size--;                             // Decrease size
    }

    while(busStopChk() == 0) {              // Wait on the STOP to be set
        if (busNACKChk() == 1) {            // If there is a NACK
            clearNACK();                    // Clear the NACK bit
            return (flt = DevFlt::kNACK);           // Fault status of "I2C_NACK"
        }

        if (busErroChk() == 1) {            // If there has been a bus error
            clearBusEr();                   // Clear the Bus error bit
            return (flt = DevFlt::kBus_Error);      // Fault status of "I2C_BUS_ERROR"
        }
    }

    clearStop();                    // Clear the STOP bit

    //Disable();                    // Ensure that the device has been disable

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    //Enable();                     // Ensure that the device has been enabled

    while(busBusyChk() == 1) {}   // Wait until the bus is no longer busy

    requestTransfer(devAddress, size, CommMode::kAutoEnd, Request::kStart_Read);
    // Setup request for the I2C bus, of the size required, and the I2C device to communicate with

    if (read(_i2c_handle_, pdata, size) != size) {      // If the requested number of bytes is not
                                                        // received
        return (flt = DevFlt::kBus_Error);              // Set fault
    }

    while(busStopChk() == 0) {              // Wait on the STOP to be set
        if (busNACKChk() == 1) {            // If there is a NACK
            clearNACK();                    // Clear the NACK bit
            return (flt = DevFlt::kNACK);           // Fault status of "I2C_NACK"
        }

        if (busErroChk() == 1) {            // If there has been a bus error
            clearBusEr();                   // Clear the Bus error bit
            return (flt = DevFlt::kBus_Error);      // Fault status of "I2C_BUS_ERROR"
        }
    }

    clearStop();                    // Clear the STOP bit

    //Disable();                    // Ensure that the device has been disable

#else
//=================================================================================================
    // So as to ensure that any downstream messages get something if this function is called,
    // return the message "No Hardware Attached" in byte steps
    for (int i = 0; i != size; i++) {
        pdata[i] = (uint8_t) _return_message_[_return_message_pointer_];
        _return_message_pointer_ = ( (_return_message_pointer_ + 1) % _message_size_ );
    }

#endif

    // Indicate that the bus is free
    comm_state = CommLock::kFree;       // Indicate bus is free

    return (flt = DevFlt::kNone);       // No fault by this point, so return no fault
}

I2CPeriph::DevFlt I2CPeriph::poleDeviceRdy(uint16_t devAddress) {
/**************************************************************************************************
 * Function will setup a communication link with the selected Device address. Where it will then
 * read back the request amount of data from the device.
 * Any errors observed with the bus during the interaction will result in exiting of the function,
 * and the class fault status being updated.
 *************************************************************************************************/
    // Indicate that the bus is not free
    comm_state = CommLock::kCommunicating;      // Indicate bus is communicating

    //Enable();                     // Ensure that the device has been enabled

    requestTransfer(devAddress, 0, CommMode::kAutoEnd, Request::kStart_Write);

    while(busStopChk() == 0) {              // Wait on the STOP to be set
        if (busNACKChk() == 1) {            // If there is a NACK
            clearNACK();                    // Clear the NACK bit
            return (DevFlt::kNACK);                 // Fault status of "I2C_NACK"
        }

        if (busErroChk() == 1) {            // If there has been a bus error
            clearBusEr();                   // Clear the Bus error bit
            return (DevFlt::kBus_Error);            // Fault status of "I2C_BUS_ERROR"
        }
    }

    clearStop();                    // Clear the STOP bit

    //Disable();                    // Ensure that the device has been disable

    // Indicate that the bus is free
    comm_state = CommLock::kFree;       // Indicate bus is free

    return(DevFlt::kNone);
}

void I2CPeriph::configTransmtIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_TXI);      // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_TXI);     // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(ktransit_data_register_empty);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(ktransit_data_register_empty);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::configTransCmIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Complete interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_TCI);      // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_TCI);     // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(ktransmit_complete);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(ktransmit_complete);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_RXI);      // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_RXI);     // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(kread_data_register_not_empty);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(kread_data_register_not_empty);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::configBusNACKIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus NACK interrupt
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_NACKI);    // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_NACKI);   // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(kbus_NACK);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(kbus_NACK);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::configBusSTOPIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus STOP interrupt
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_STOPI);    // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_STOPI);   // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(kbus_stop);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(kbus_stop);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::configBusErroIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus Error interrupt
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_I2C_ENABLE_IT(_i2c_handle_, I2C_IT_ERRI);     // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_I2C_DISABLE_IT(_i2c_handle_, I2C_IT_ERRI);    // Then disable interrupt
    }


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(kbus_error);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(kbus_error);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void I2CPeriph::intMasterReq(uint16_t devAddress, uint16_t size, uint8_t *Buff,
                             CommMode mode, Request reqst,
                             volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function will be called to start off a new I2C communication.
 * This version of the I2C Form update functions, is expected to be only used for "Interrupt"
 * based communication, and therefore uses the "GenBuffer" handle input.
 *************************************************************************************************/
    // Put request into Form Queue
    specificRequest(devAddress,
                          size,
                          Buff,
                          mode, reqst,
                          fltReturn, cmpFlag
                         );

    // Trigger interrupt(s)
    startInterrupt();
}

void I2CPeriph::startInterrupt(void) {
/**************************************************************************************************
 * Function will be called to start off a new I2C communication if there is something in the
 * queue, and the bus is free.
 *************************************************************************************************/
    if ( (comm_state == CommLock::kFree) && (_form_queue_.state() != kGenBuffer_Empty) ) {
        // If the I2C bus is free, and there is I2C request forms in the queue
        _form_queue_.outputRead( &(_cur_form_) );           // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_form_.Flt) != DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_queue_.state() == kGenBuffer_Empty ) {   // If buffer is empty, break out
                //Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_queue_.outputRead( &(_cur_form_) );           // Capture form request
        }

        comm_state = CommLock::kCommunicating;      // Lock I2C bus

        //Enable();

        requestTransfer(  _cur_form_.devAddress,
                (uint8_t) _cur_form_.size,
                          _cur_form_.Mode,
                          _cur_form_.Reqst
                        );
            // Trigger communication as per Form request

        if (_cur_reqst_ == Request::kStart_Write) {     // If this is a write request
            configTransmtIT(InterState::kIT_Enable);    // Then enable Transmit Empty buffer
        }                                               // interrupt

        else if (_cur_reqst_ == Request::kStart_Read) { // If this is a read request
            configReceiveIT(InterState::kIT_Enable);    // Then enable Receive buffer full
        }                                               // interrupt
    }
    else if ( (comm_state == CommLock::kFree) && (_form_queue_.state() == kGenBuffer_Empty) ) {
        //Disable();
    }
}

void I2CPeriph::intReqFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form, to indicate how much data has been completed.
 * Indicate that the I2C Device is now free for any new communication
 * Disables the Receive/Transmit Interrupts
 *************************************************************************************************/
    *(_cur_form_.Cmplt)  += (_cur_form_.size - _cur_count_);
    // Indicate how many data points have been transfered (curCount should be 0)

    // Indicate that I2C bus is now free, and disable any interrupts
    comm_state = CommLock::kFree;

    configReceiveIT(InterState::kIT_Disable);   // Disable Receive buffer full interrupt
    configTransmtIT(InterState::kIT_Disable);   // Disable Transmit empty buffer interrupt
}

void I2CPeriph:: handleEventIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the I2C events within the I2C class.
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
 *      Transmit Complete Flag
 *          - This functionality is not supported, however this status should be used if the
 *            "Auto" communication has not been selected. Such that when the current pack of 255
 *            bytes to transmit has been done, completed then the same communication can continue
 *            with a new set of data.
 *            For communicating packets of data > 255bytes.
 *            Note - Will be cleared by hardware, when either a START or STOP has been triggered.
 *
 *      Transmit Buffer Empty
 *          - Used to pull out more data from the current Request form, and put onto the Transmit
 *            hardware buffer.
 *            Note - Writing to the hardware will clear this status bit.
 *
 *      Receive Buffer full (not empty)
 *          - Similar to the Transmit Buffer Empty interrupt, this interrupt will be used to read
 *            from the hardware the current read value. This will then be passed to the requested
 *            data location (as per Request form).
 *
 *      I2C Stop has been set
 *          - If a STOP has been detected, then current Request form has been completed.
 *            Will then indicate how much data has been transmitted, and look to see if there is
 *            another other requests. If none, then communication interrupts are disabled.
 *
 *      I2C data has not been acknowledged (NACK)
 *          - If a NACK is detected during the communication with the external device. Then the
 *            current Request Form is binned. A fault is provided to the requested source
 *            location, and the return complete indication is setup to show how many bytes had
 *            been transmitted prior to NACK.
 *            Source functional will then need to determine what to do in this situation.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
 
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the I2C events within the I2C class.
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
 *      I2C data has not been acknowledged (NACK)
 *          - If a NACK is detected during the communication with the external device. Then the
 *            current Request Form is binned. A fault is provided to the requested source
 *            location, and the return complete indication is setup to show how many bytes had
 *            been transmitted prior to NACK.
 *            Source functional will then need to determine what to do in this situation.
 *
 *      Receive Buffer full (not empty)
 *          - Similar to the Transmit Buffer Empty interrupt, this interrupt will be used to read
 *            from the hardware the current read value. This will then be passed to the requested
 *            data location (as per Request form).
 *
 *      Transmit Buffer Empty
 *          - Used to pull out more data from the current Request form, and put onto the Transmit
 *            hardware buffer.
 *            Note - Writing to the hardware will clear this status bit.
 *
 *      Transmit Complete Flag
 *          - This functionality is not supported, however this status should be used if the
 *            "Auto" communication has not been selected. Such that when the current pack of 255
 *            bytes to transmit has been done, completed then the same communication can continue
 *            with a new set of data.
 *            For communicating packets of data > 255bytes.
 *            Note - Will be cleared by hardware, when either a START or STOP has been triggered.
 *
 *      I2C Stop has been set
 *          - If a STOP has been detected, then current Request form has been completed.
 *            Will then indicate how much data has been transmitted, and look to see if there is
 *            another other requests. If none, then communication interrupts are disabled.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    uint8_t temp_DR = 0;

    if ( (busNACKChk() & busNACKITChk()) == 0x01) { // If I2C NACK received
        //clearStop();                                // Clear the STOP status
        //clearNACK();                                // Clear the NACK status
        *(_cur_form_.Flt) = DevFlt::kNACK;          // Set requested fault flag

        // Flush the contents of the Transmit buffer
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //__HAL_I2C_CLEAR_FLAG(_i2c_handle_, I2C_FLAG_TXE);

        //intReqFormCmplt();          // Complete the current request form
        //startInterrupt();           // Check if any new requests remain
    }

    else if ( (receiveToReadChk() & receiveToReadITChk()) == 0x01) {
        // If Receive Buffer full triggered
        temp_DR  = readDR();
        // Retrieve data from the hardware

        // Only when the class global count is not zero. Put the read data into the requested data
        // location as per I2C Request Form
        if (_cur_count_ != 0) {
            putFormReadData( &(_cur_form_) , temp_DR );

            _cur_count_--;      // Decrement the class global current count
        }
    }

    else if ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01) {
        // If Transmit Buffer Empty triggered
        // Only update the transmit hardware buffer, if the class global count is not zero
        if (_cur_count_ == 0)   // If count is equal to zero
            temp_DR     = 0x00; // Populate with zeroes (default data)

        else {
            temp_DR  = getFormWriteData( &(_cur_form_) );
            // Retrieve next data point from the Request I2C Form, and put onto hardware queue

            _cur_count_--;      // Decrement the class global current count
        }
        // Get data from form, and put into hardware buffer

        writeDR(temp_DR);
    }

    else if ( (transmitComptChk() & transmitComptITChk()) == 0x01) {
        // If Transmit Complete triggered
        // Not really supported yet!
    }

    if ( (busStopChk() & busStopITChk()) == 0x01) { // If I2C Stop triggered
        clearStop();                                // Clear the STOP status
        clearNACK();                                // Clear the NACK status

        // Flush the contents of the Transmit buffer
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          __HAL_I2C_CLEAR_FLAG(_i2c_handle_, I2C_FLAG_TXE);

        intReqFormCmplt();          // Complete the current request form
        startInterrupt();           // Check if any new requests remain
    }
    
#elif ( (zz__MiEmbedType__zz == 10) || (zz__MiEmbedType__zz ==  0)  )
// Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As setting up a real interrupt for Raspberry Pi involves hacking the kernel, which I am not
 *  doing, the route I have taken is to use threads - pthreads.
 *  What this involves is creating a separate stream (thread) which will just check the size of the
 *  I2C peripheral buffer or if data has been requested to be sent (via pseudo interrupt
 *  register). Then the main thread can use the Read/Transmit functions as normal.
 *
 *  So setting up the pthread:
 *  The -pthread needs to be added to the G++ linker library, won't require the "-" within eclipse,
 *  however for "CMakeList", will need to be written as "-lpthread".
 *  Then within the main code add "include <pthread.h>
 *
 *  This will then allow you to create a new stream:
 *  main.c {
 *  pthread_t thread;   // Create thread handle
 *  if (pthread_create(&thread, NULL, &threadfunction, NULL) != 0) {
 *      std::cout << "Failed to create thread" << std::endl;
 *  }
 *
 * pthread_create will return 0 if it has created a new thread. In the example above "&thread" is
 * the thread handle created above, next entry ...no idea... 3rd entry is a pointer to the desired
 * function call, 4th can be used to provide values to the function - however I haven't tried this.
 *
 * void *threadfunction(void *value) {
 *  uint8_t I2CReturn;
 *  while (1) {
 *      delay(100);
 *      MyI2C->handleIRQ();
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the I2CPeriph will need to be made global to allow this
 * new thread to all the "IRQHandler"
 *************************************************************************************************/
    if ( (transmitComptChk() & transmitComptITChk()) == 0x01) {
        // If Transmit Complete triggered
        // Not really supported yet!
    }

    if ( ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01 )  ||
         ( (receiveToReadChk() & receiveToReadITChk()) == 0x01 ) ) {
        // If there is any new transmit/read requests, then ...

#if   (zz__MiEmbedType__zz == 10)
        if      (_cur_form_.Reqst == Request::kStart_Read) {
            if (read(_i2c_handle_, _cur_form_.Buff, _cur_form_.size) != _cur_form_.size) {
                // If the requested number of bytes is not received, set Bus Error fault
                *(_cur_form_.Flt) = DevFlt::kBus_Error;
            }
        }

        else if (_cur_form_.Reqst == Request::kStart_Write) {
            if (write(_i2c_handle_, _cur_form_.Buff, _cur_form_.size) != _cur_form_.size) {
                // If the requested number of bytes is not transmitted, set Bus Error fault
                *(_cur_form_.Flt) = DevFlt::kBus_Error;
            }
        }

        else {
            errorMessage("Requested mode not supported in interrupt mode");
        }

#else
        if      (_cur_form_.Reqst == Request::kStart_Read) {
            for (int i = 0; i != _cur_form_.size; i++) {
                _cur_form_.Buff[i] = (uint8_t) _return_message_[_return_message_pointer_];
                _return_message_pointer_ = ( (_return_message_pointer_ + 1) % _message_size_ );
            }
        }

#endif


        _cur_count_ = 0;            // Force count to '0', as all data has been transmitted

        intReqFormCmplt();          // Complete the current request form
        startInterrupt();           // Check if any new requests remain
    }

    // As the RaspberryPi/Default functions do not have any detection of bus errors, the following
    // function calls will never be used; however have been left within the function so as to
    // allow for this feature in the future if required

    if ( (busStopChk() & busStopITChk()) == 0x01) { // If I2C Stop triggered
        clearStop();                                // Clear the STOP status
        clearNACK();                                // Clear the NACK status

        intReqFormCmplt();          // Complete the current request form
        startInterrupt();           // Check if any new requests remain
    }

    if ( (busNACKChk() & busNACKITChk()) == 0x01) { // If I2C NACK received
        clearNACK();                                // Clear the NACK status
        *(_cur_form_.Flt) = DevFlt::kNACK;          // Set requested fault flag

        intReqFormCmplt();          // Complete the current request form
        startInterrupt();           // Check if any new requests remain
    }
    
#else
//=================================================================================================

#endif
}

void I2CPeriph:: handleErrorIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the I2C errors within the I2C class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occurred, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:
 *      Bus Error
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    if ( (busErroChk() & busErrorITChk()) == 0x01) {// If Bus Error triggered
        clearBusEr();                               // Clear the Bus Error
        *(_cur_form_.Flt)    = DevFlt::kBus_Error;  // Set requested fault flag
    }
}

I2CPeriph::~I2CPeriph() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    if (close(_i2c_handle_) < 0)
        errorMessage("Error, unable to close I2C device - %s", _device_loc_);

#else
//=================================================================================================

#endif
}
