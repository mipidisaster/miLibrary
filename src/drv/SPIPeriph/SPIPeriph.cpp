/**************************************************************************************************
 * @file        SPIPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic SPIPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_SPIPe__HD               // Header for SPI

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL UART library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL UART library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#include <stdio.h>                      // Standard I/O - including the error file
#include <stdarg.h>                     // Allows functions to accept an indefinite number of
                                        // arguments
#include <stdlib.h>                     // Needed for 'exit'

#include <fcntl.h>                      // Needed for SPI port
#include <unistd.h>                     //
#include <sys/ioctl.h>                  // Control of I/O devices
#include <linux/spi/spidev.h>           // ioctl SPI specific parameters
// See  https://raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface
//      https://kernel.org/doc/Documentation/spi/spidev
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
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_DeMux__HD

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

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
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
#else   // Raspberry Pi or Default build configuration
//=================================================================================================
SPIPeriph::SPIPeriph(const char *deviceloc, int speed, SPIMode Mode, uint16_t FormSize) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for RaspberryPi, to simplify the RaspberryPi version (and as
 * size is less of a constraint) function will create arrays for the internal Form buffers.
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    popGenParam();              // Populate generic class parameters

    _mode_              = Mode;         // Copy across the selected Mode
    _pseudo_interrupt_  = 0x00;         // pseudo interrupt register used to control the SPI
                                        // interrupt for Raspberry Pi
    _device_loc_        = deviceloc;    // Capture the folder location of SPI device
    _spi_speed_         = speed;

    _form_queue_.create(new Form[FormSize], FormSize);

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    int tempMode = 0;
    if      (_mode_ == SPIMode::kMode1) // If Mode 1 is selected then
        tempMode = 1;                   // Store "1"
    else if (_mode_ == SPIMode::kMode2) // If Mode 2 is selected then
        tempMode = 2;                   // Store "2"
    else if (_mode_ == SPIMode::kMode3) // If Mode 3 is selected then
        tempMode = 3;                   // Store "3"
    else                                // If any other Mode is selected then
        tempMode = 0;                   // Default to "0"

    _spi_handle_        = open(_device_loc_, O_RDWR);   // Open file to the SPI Device

    if (_spi_handle_ < 0)
        errorMessage("Unable to open SPI device: %s", _device_loc_);

    if (ioctl (_spi_handle_,  SPI_IOC_WR_MODE, &tempMode)                       < 0)
        errorMessage("SPI Mode Change failure");

    uint8_t tempBPW = kSPI_bits_per_word;

    if (ioctl (_spi_handle_,  SPI_IOC_WR_BITS_PER_WORD, &tempBPW)               < 0)
        errorMessage("SPI BPW Change failure");

    if (ioctl (_spi_handle_,  SPI_IOC_WR_MAX_SPEED_HZ, &_spi_speed_)            < 0)
        errorMessage("SPI Speed Change failure, cannot set speed to %dHz", _spi_speed_);

#else
    _spi_handle_                = 0;
    _return_message_            = {"No SPI Hardware Attached"};
    _message_size_              = 24;
    _return_message_pointer_    = 0;

#endif
}

SPIPeriph::SPIPeriph(const char *deviceloc, int speed, SPIMode Mode,
                                            Form *FormArray, uint16_t FormSize) {
/**************************************************************************************************
 * Create a SPIPeriph class specific for RaspberryPi
 *  This version requires pointers to the Write/Read UART Form system and sizes
 * Receives the desired channel, speed and Mode
 *************************************************************************************************/
    popGenParam();              // Populate generic class parameters

    _mode_              = Mode;         // Copy across the selected Mode
    _pseudo_interrupt_  = 0x00;         // pseudo interrupt register used to control the SPI
                                        // interrupt for Raspberry Pi
    _device_loc_        = deviceloc;    // Capture the folder location of SPI device
    _spi_speed_         = speed;

    _form_queue_.create(FormArray, FormSize);

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    int tempMode = 0;
    if      (_mode_ == SPIMode::kMode1) // If Mode 1 is selected then
        tempMode = 1;                   // Store "1"
    else if (_mode_ == SPIMode::kMode2) // If Mode 2 is selected then
        tempMode = 2;                   // Store "2"
    else if (_mode_ == SPIMode::kMode3) // If Mode 3 is selected then
        tempMode = 3;                   // Store "3"
    else                                // If any other Mode is selected then
        tempMode = 0;                   // Default to "0"

    _spi_handle_        = open(_device_loc_, O_RDWR);   // Open file to the SPI Device

    if (_spi_handle_ < 0)
        errorMessage("Unable to open SPI device: %s", deviceloc);

    if (ioctl (_spi_handle_,  SPI_IOC_WR_MODE, &tempMode)                       < 0)
        errorMessage("SPI Mode Change failure");

    uint8_t tempBPW = kSPI_bits_per_word;

    if (ioctl (_spi_handle_,  SPI_IOC_WR_BITS_PER_WORD, &tempBPW)               < 0)
        errorMessage("SPI BPW Change failure");

    if (ioctl (_spi_handle_,  SPI_IOC_WR_MAX_SPEED_HZ, &_spi_speed_)            < 0)
        errorMessage("SPI Speed Change failure, cannot set speed to %dHz", _spi_speed_);

#else
    _return_message_ = {"No SPI Hardware Attached"};
    _message_size_ = 24;
    _return_message_pointer_ = 0;

#endif
}

void SPIPeriph::errorMessage(const char *message, ...) {
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

uint8_t SPIPeriph::dataWriteRead(uint8_t *wData, uint8_t *rData, int len)
/**************************************************************************************************
 * Wrapper for the 'wiringPi' function 'wiringPiSPIDataRW'
 *************************************************************************************************/
{
#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
//=================================================================================================
    struct spi_ioc_transfer spi = { 0 };

    spi.tx_buf          = (unsigned long) wData;        // Construct SPI hardware parameters
    spi.rx_buf          = (unsigned long) rData;
    spi.len             = len;
    spi.delay_usecs     = kSPI_delay;
    spi.speed_hz        = _spi_speed_;
    spi.bits_per_word   = kSPI_bits_per_word;

    return (  ioctl (_spi_handle_,  SPI_IOC_MESSAGE(1), &spi)  );

#else
//=================================================================================================
    // So as to ensure that any downstream messages get something if this function is called,
    // return the message "No Hardware Attached" in byte steps
    for (int i = 0; i != len; i++) {
        rData[i] = (uint8_t) _return_message_[_return_message_pointer_];
        _return_message_pointer_ = ( (_return_message_pointer_ + 1) % _message_size_ );
    }

    return ( 0 );   // Returnt no fault with read
#endif
}

void SPIPeriph::pseudoRegisterSet(uint8_t *pseudoregister, uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function set the desired 'entry' of the input 'pseudoregister'
 *************************************************************************************************/
    *pseudoregister |= entry;
}

void SPIPeriph::pseudoRegisterClear(uint8_t *pseudoregister, uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function clear the desired 'entry' of the input 'pseudoregister'
 *************************************************************************************************/
    *pseudoregister &= ~(entry);
}

uint8_t SPIPeriph::pseudoStatusChk(uint8_t pseudoregister, uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function see if the desired 'entry' of the input 'pseudoregister' is set
 *************************************************************************************************/
    return ( pseudoregister & entry );
}

#endif

void SPIPeriph::enable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    __HAL_SPI_ENABLE(_spi_handle_);     // Enable the SPI interface

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_SPI_ENABLE(_spi_handle_);     // Enable the SPI interface

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Do nothing, as there is no hardware to enable/disable (as calling the read/write function
// covers this)

#endif
}

void SPIPeriph::disable(void) {
/**************************************************************************************************
 * Enable the SPI Device
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    __HAL_SPI_DISABLE(_spi_handle_);    // Disable the SPI interface

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_SPI_DISABLE(_spi_handle_);    // Disable the SPI interface

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Do nothing, as there is no hardware to enable/disable (as calling the read/write function
// covers this)

#endif
}

uint8_t SPIPeriph::readDR(void) {
/**************************************************************************************************
 * Read from the SPI hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    return ((uint8_t) _spi_handle_->Instance->DR);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// STM32L4 uses a RXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)
    return( (uint8_t) _spi_handle_->Instance->DR );

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library or Default configuration
// doesn't need this . Function will not be called by upper level functions
    return 0;

#endif
}

void SPIPeriph::writeDR(uint8_t data) {
/**************************************************************************************************
 * Write to the SPI hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    _spi_handle_->Instance->DR = data;

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// STM32L4 uses a TXFIFO of 32bits (4 x 8bits), this function will only work if the hardware has
// been configured to allow a read of only 8bits (or less)
// To ensure only 8bits is transmitted, need to ensure that we cast the Data Register (DR) to
// unsigned 8bits, hence the initial casing
    *(uint8_t *)&_spi_handle_->Instance->DR = data;

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Unable to get to this level of granularity using the wiringPi library or Default configuration
// doesn't need this . Function will not be called by upper level functions
// Do nothing!

#endif
}

uint8_t SPIPeriph::transmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t SPIPeriph::receiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t SPIPeriph::busBusyChk(void) {
/**************************************************************************************************
 * Check to see if the SPI bus is already communicating (if bus is busy, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_BSY) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_BSY) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (bus isn't busy), such that
                                        // any downstream will continue as if entry is set

#endif
}

uint8_t SPIPeriph::busOverRunChk(void) {
/**************************************************************************************************
 * Check to see if a Bus Over run has occurred (if over run, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_OVR) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_OVR) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (bus overrun not occurred),
                                        // such that any downstream will continue as if entry is
                                        // set

#endif
}

void SPIPeriph::clearBusOvrRun(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Overrun fault state
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// To clear the Bus Over run status form STM32F, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(_spi_handle_);      // Utilise the existing MACRO

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// To clear the Bus Over run status form STM32L, the DR register needs to be read, followed by a
// read of the Status Register.
    __HAL_SPI_CLEAR_OVRFLAG(_spi_handle_);      // Utilise the existing MACRO


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
}

uint8_t SPIPeriph::busModeFltChk(void) {
/**************************************************************************************************
 * Check to see if a Bus mode fault has occurred (if mode fault, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_MODF) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_FLAG(_spi_handle_, SPI_FLAG_MODF) != 0x00 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (0);                         // Return a positive output (bus fault not occurred),
                                        // such that any downstream will continue as if entry is
                                        // set

#endif
}

void SPIPeriph::clearBusModeFlt(void) {
/**************************************************************************************************
 * Go through the required sequence to clear the Bus Mode fault state
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(_spi_handle_);     // Utilise the existing MACRO

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// Make a read access to the Status Register, then write to the CR1 register.
// Note with this fault the SPI peripheral will be disabled.
    __HAL_SPI_CLEAR_MODFFLAG(_spi_handle_);     // Utilise the existing MACRO


#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
}

uint8_t SPIPeriph::transmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_TXE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_TXE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(_pseudo_interrupt_, ktransit_data_register_empty) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t SPIPeriph::receiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_RXNE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(_pseudo_interrupt_, kread_data_register_not_empty) != 0 )
        return (1);
    else
        return (0);

#endif
}

uint8_t SPIPeriph::busErrorITChk(void) {
/**************************************************************************************************
 * Check to see whether the Bus Error interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_ERR) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_SPI_GET_IT_SOURCE(_spi_handle_, SPI_IT_ERR) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if ( pseudoStatusChk(_pseudo_interrupt_, kbus_error) != 0 )
        return (1);
    else
        return (0);

#endif
}

SPIPeriph::CSHandle SPIPeriph::hardwareCS(void) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via the Hardware Chip Select (no software interaction required)
 *************************************************************************************************/
    CSHandle temp_struct = { .GPIO_CS = __null,
                             .Type = CSHandle::CSType::kHardware_Managed
    };

    return (temp_struct);                               // Return build structure
}

SPIPeriph::CSHandle SPIPeriph::softwareGPIO(GPIO *CS) {
/**************************************************************************************************
 * Generate a CSHandle struct variable with the parameters configured to indicate that SPI device
 * communication is to be done via software selection of input GPIO pin.
 *************************************************************************************************/
    CSHandle temp_struct = {.GPIO_CS = CS,
                            .Type    = CSHandle::CSType::kSoftware_GPIO
    };

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
    Form request_form = {               // Initialise the "Form" variable with input
            .devLoc     = devLoc,
            .size       = size,
            .TxBuff     = __null,
            .RxBuff     = __null,
            .Cmplt      = cmpFlag,
            .Flt        = fltReturn
    };

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

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
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

#else   // Raspberry Pi or Default build configuration
//=================================================================================================

    dataWriteRead(wData, rData, size);         // Transfer data

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

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_TXE);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_TXE); // Then disable interrupt
    }

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(&_pseudo_interrupt_, ktransit_data_register_empty);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }
    else {                                                  // If request is to disable
        pseudoRegisterClear(&_pseudo_interrupt_, ktransit_data_register_empty);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register
    }

#endif
}

void SPIPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_RXNE); // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_RXNE);// Then disable interrupt
    }

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(&_pseudo_interrupt_, kread_data_register_not_empty);
        // Enable the pseudo Receive bit - via the "Pseudo interrupt" register
    }
    else {                                                      // If request is to disable
        pseudoRegisterClear(&_pseudo_interrupt_, kread_data_register_not_empty);
        // Disable the pseudo Receive bit - via the "Pseudo interrupt" register
    }

#endif
}

void SPIPeriph::configBusErroIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Bus Error interrupt
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {               // If request is to enable
        __HAL_SPI_ENABLE_IT(_spi_handle_, SPI_IT_ERR);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_SPI_DISABLE_IT(_spi_handle_, SPI_IT_ERR); // Then disable interrupt
    }

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(&_pseudo_interrupt_, kbus_error);
        // Enable the pseudo Bus error bit - via the "Pseudo interrupt" register
    }
    else {                                                      // If request is to disable
        pseudoRegisterClear(&_pseudo_interrupt_, kbus_error);
        // Disable the pseudo Bus error bit - via the "Pseudo interrupt" register
    }

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
 * Interrupt Service Routine for the SPI class. Each of the supported devices needs to call this
 * function in different ways - therefore each implementation is mentioned within the coded
 * section.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occurred, and the interrupt is enabled, then this
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
 *************************************************************************************************/

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
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

#if   (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
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

#elif ( (zz__MiEmbedType__zz == 10) || (zz__MiEmbedType__zz ==  0)  )
// Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As setting up a real interrupt for Raspberry Pi involves hacking the kernel, which I am not
 *  doing, the route I have taken is to use threads - pthreads.
 *  What this involves is creating a separate stream (thread) which will just check the size of the
 *  SPI peripheral buffer or if data has been requested to be sent (via pseudo interrupt
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
 *  uint8_t SPIReturn;
 *  while (1) {
 *      delay(100);
 *      MySPI->handleIRQ();
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the SPIPeriph will need to be made global to allow this
 * new thread to all the "IRQHandler"
 *************************************************************************************************/
    // As there is no separate functions for the RaspberryPi/Default setup for the reading and
    // writing of hardware - there is only the one function which covers both - 'dataWriteRead'.
    // Therefore have been combined into a single function call...

    while (CommState == CommLock::kCommunicating) {
        // While there is data to be transmitted (MASTER MODE), then...

        if ( ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01 )  ||
             ( (receiveToReadChk() & receiveToReadITChk()) == 0x01 ) ) {
            // If there is any new transmit/read requests, then ...
            dataWriteRead(_cur_form_.TxBuff, _cur_form_.RxBuff, _cur_form_.size);
            // Will need to improve at some point, such that it will read whether the data has been
            // transfered or not. It is assumed here that it will ALWAYS work.

            _cur_count_ = 0;    // Force count to '0', as all data as been transferred.
            configTransmtIT(InterState::kIT_Disable);   // Disable 'interrupt'
            intReqFormCmplt();          // Complete the current request form (no faults)
            startInterrupt();           // Check if any new requests remain
        }}


    // As the RaspberryPi/Default functions do not have any detection of bus errors, the following
    // function calls will never be used; however have been left within the function so as to
    // allow for this feature in the future if required

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

#else
//=================================================================================================

#endif
}

SPIPeriph::~SPIPeriph() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    if (close(_spi_handle_) < 0)
        errorMessage("Error, unable to close SPI device - %s", _device_loc_);

#else
//=================================================================================================

#endif
}

