/**************************************************************************************************
 * @file        UART.cpp
 * @author      Thomas
 * @brief       Source file for the Generic UART Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_USART__HD               // Header for UART

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

#include <fcntl.h>                      // Needed for UART port
#include <unistd.h>                     //
#include <sys/ioctl.h>                  // Control of I/O devices
#include <termios.h>                    // Defines baudrate settings and 'speed_t'

// See  https://raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
//      https://kernel.org/doc/Documentation/serial/driver.rst
//          https://linux.die.net/
//      http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html

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

void UARTPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the UARTPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    flt           = DevFlt::kInitialised;   // Initialise the fault to "initialised"
    wrte_comm_state = CommLock::kFree;      // Indicate bus is free (Write)
    read_comm_state = CommLock::kFree;      // Indicate bus is free (Read)

    _cur_wrte_count_  = 0;          // Initialise the current packet size count
    _cur_wrte_form_   = { 0 };      // Initialise the form to a blank entry

    _cur_read_count_  = 0;          // Initialise the current packet size count
    _cur_read_form_   = { 0 };      // Initialise the form to a blank entry
}

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
UARTPeriph::UARTPeriph(UART_HandleTypeDef *UART_Handle,
                       Form *WrteForm, uint16_t WrteFormSize,
                       Form *ReadForm, uint16_t ReadFormSize) {
/**************************************************************************************************
 * Create a UARTPeriph class specific for the STM32 device
 * Receives the address of the UART Handle of device - generated from cubeMX
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired I2C
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    popGenParam();                      // Populate generic class parameters
    _uart_handle_ = UART_Handle;        // Copy data into class

    _form_wrte_q_.create(WrteForm, WrteFormSize);
    _form_read_q_.create(ReadForm, ReadFormSize);
}

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
speed_t UARTPeriph::configBaudrate(int baud) {
/**************************************************************************************************
 * Simple function to take the input integer value, and select the correct baudrate entry.
 * Replicated from wiringSerial.c for simplicity
 *************************************************************************************************/
    speed_t myBaud ;

    switch (baud)
    {
      case      50:   myBaud =      B50 ; break ;
      case      75:   myBaud =      B75 ; break ;
      case     110:   myBaud =     B110 ; break ;
      case     134:   myBaud =     B134 ; break ;
      case     150:   myBaud =     B150 ; break ;
      case     200:   myBaud =     B200 ; break ;
      case     300:   myBaud =     B300 ; break ;
      case     600:   myBaud =     B600 ; break ;
      case    1200:   myBaud =    B1200 ; break ;
      case    1800:   myBaud =    B1800 ; break ;
      case    2400:   myBaud =    B2400 ; break ;
      case    4800:   myBaud =    B4800 ; break ;
      case    9600:   myBaud =    B9600 ; break ;
      case   19200:   myBaud =   B19200 ; break ;
      case   38400:   myBaud =   B38400 ; break ;
      case   57600:   myBaud =   B57600 ; break ;
      case  115200:   myBaud =  B115200 ; break ;
      case  230400:   myBaud =  B230400 ; break ;
      case  460800:   myBaud =  B460800 ; break ;
      case  500000:   myBaud =  B500000 ; break ;
      case  576000:   myBaud =  B576000 ; break ;
      case  921600:   myBaud =  B921600 ; break ;
      case 1000000:   myBaud = B1000000 ; break ;
      case 1152000:   myBaud = B1152000 ; break ;
      case 1500000:   myBaud = B1500000 ; break ;
      case 2000000:   myBaud = B2000000 ; break ;
      case 2500000:   myBaud = B2500000 ; break ;
      case 3000000:   myBaud = B3000000 ; break ;
      case 3500000:   myBaud = B3500000 ; break ;
      case 4000000:   myBaud = B4000000 ; break ;

      default:
        errorMessage("UART Speed Change failure, cannot set speed to %dHz", baud);
        return -2 ;
    }

    return myBaud;
}

UARTPeriph::UARTPeriph(const char *deviceloc, int baud,
                       Form *WrteForm, uint16_t WrteFormSize,
                       Form *ReadForm, uint16_t ReadFormSize) {
/**************************************************************************************************
 * Create a UART class specific for the Raspberry Pi
 *  This version requires pointers to the Write/Read UART Form system and sizes
 *
 *  This will then open up the serial interface, and configure a "pseudo_interrutp" register, so
 *  as to provide the Raspberry Pi the same function use as other embedded devices.
 *  The Receive and Transmit buffers size will be as per input "BufferSize"
 *************************************************************************************************/
    popGenParam();                      // Populate generic class parameters

    _device_loc_        = deviceloc;    // Capture the folder location of UART device
    _baud_rate_         = baud;         // Capture the desired baud rate
    _pseudo_interrupt_  = 0x00;         // pseudo interrupt register used to control the UART
                                        // interrupt for Raspberry Pi

    // Configure both the Write and Read Buffers to be the size as per input
    _form_wrte_q_.create(WrteForm, WrteFormSize);
    _form_read_q_.create(ReadForm, ReadFormSize);

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    /* Open modem device for reading and writing and not as controlling tty because we don't want
     * to get killed if linenoise sends CTRL-C!
     *
     * O_RWDR       -Open for reading and writing
     * O_NOCTTY     - If set and path identifies a terminal device, open() will not cause the
     *                terminal device to become the controlling terminal for the process
     * O_NDELAY     - Enables nonblocking mode. When set read requests on the file can return
     *                immediately with a failure status if there is no input immediately available
     *                (instead of blocking). Likewise, write requests can also return immediately
     *                with a failure status if the output can't be written immediately.
     *                (same as O_NONBLOCK, used both as a 'just in case').
     */
    _uart_handle_ = open(deviceloc, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

    if (_uart_handle_ < 0)
        errorMessage("Unable to open UART device: %s", deviceloc);
    // Serial interface should now be open.
    // Just need to confirm the interface now...
    // For information see - http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html

    struct termios options;     // termios structure for configuring terminal
    tcgetattr(_uart_handle_, &options);     // Retrieve current configuration.

    cfmakeraw(&options);    // Will clean up the c_i/_o/_l flags to a 'default' configuration
                            // Search to find this (linux.die.net)
    cfsetispeed(&options, configBaudrate(_baud_rate_));
    cfsetospeed(&options, configBaudrate(_baud_rate_));

    /* 'c_cflag' fields describes the hardware control of the terminal
     *  CS8         - 8bits
     *  CREAD       - Enable Receive
     *  CLOCAL      - Ignore modem status lines (included as was in example used)
     */
    options.c_cflag |= (CLOCAL | CREAD | CS8 | configBaudrate(_baud_rate_));
    //options.c_cflag |= (CLOCAL | CREAD | CS8 );

    /* 'c_iflag' field describes the basic terminal input control
     *      - NONE USED -
     */
    options.c_iflag = 0;

    /* 'c_oflag' field specifies the system treatment of output
     *      - NONE USED -
     */
    options.c_oflag = 0;

    /* 'c_lflag' field of the argument structure is used to control various terminal functions:
     *      - NONE USED -
     */
    options.c_lflag = 0;

    /* 'c_cc' control characters
     * VMIN     = Minimum value
     * VTIME    = Time
     */
    options.c_cc[VMIN]  =   0;
    options.c_cc[VTIME] = 100;  // 100deciseconds

    // Clean the modem line and activate the settings for port
    tcflush(_uart_handle_, TCIFLUSH);               // Flush
    tcsetattr(_uart_handle_, TCSANOW, &options);    // Enter settings

#else
    _return_message_ = {"No USART Hardware Attached"};
    _message_size_ = 26;
    _return_message_pointer_ = 0;

#endif
}

void UARTPeriph::errorMessage(const char *message, ...) {
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

int  UARTPeriph::anySerDataAvil(void) {
/**************************************************************************************************
 * RaspberryPi specific function to determine amount of data within the hardware
 *************************************************************************************************/
#if   (zz__MiEmbedType__zz == 10)       // If configured for RaspberryPi, then use wiringPi
    int return_value;

    // FIONREAD - returns the number of bytes that are immediately available for reading (ioctl)
    // https://freebsd.org/cgi/man.cgi?sektion=2&query=ioctl
    // Slightly better/useful source -
    //  https://linux.die.net/man/4/tty_ioctl (as this is the terminal version of ioctl)
    if ( ioctl (_uart_handle_,  FIONREAD, &return_value) == -1 )
        return -1;

    return return_value;

#elif (zz__MiEmbedType__zz ==  0)       // If configured for No Hardware then
    return ((int) 1);                   // Return '1' so as to ensure that no hardware message is
                                        // captured
#endif
}

void UARTPeriph::pseudoRegisterSet(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function set the desired 'entry' of '_pseudo_interrupt_'
 *************************************************************************************************/
    _pseudo_interrupt_  |= entry;
}

void UARTPeriph::pseudoRegisterClear(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function clear the desired 'entry' of '_pseudo_interrupt_'
 *************************************************************************************************/
    _pseudo_interrupt_  &= ~(entry);
}

uint8_t UARTPeriph::pseudoStatusChk(uint8_t entry) {
/**************************************************************************************************
 * RaspberryPi specific function see if the desired 'entry' of '_pseudo_interrupt_' is set
 *************************************************************************************************/
    return ( _pseudo_interrupt_ & entry );
}

#endif

uint8_t UARTPeriph::readDR(void) {
/**************************************************************************************************
 * Read from the UART hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    return ((uint8_t)_uart_handle_->Instance->DR);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    return ((uint8_t)_uart_handle_->Instance->RDR);

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    uint8_t x;

    if (read (_uart_handle_, &x, 1) != 1)
        return -1;

    return ( (uint8_t) x ) & 0xFF;

#else
//=================================================================================================
    // So as to ensure that any downstream messages get something if this function is called,
    // return the message "No Hardware Attached" in byte steps
    uint8_t temp_char = (uint8_t) _return_message_[_return_message_pointer_];

    _return_message_pointer_ = ( (_return_message_pointer_ + 1) % _message_size_ );

    return ( temp_char );
#endif
}

void UARTPeriph::writeDR(uint8_t data) {
/**************************************************************************************************
 * Write from the UART hardware
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    _uart_handle_->Instance->DR = data;

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    _uart_handle_->Instance->TDR = data;

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    write(_uart_handle_, &data, 1);

#else
//=================================================================================================
// Do nothing

#endif
}

uint8_t UARTPeriph::transmitEmptyChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit buffer (if empty, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_TXE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t UARTPeriph::transmitComptChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Transmit communication (if completed, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_TC) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_TC) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

uint8_t UARTPeriph::receiveToReadChk(void) {
/**************************************************************************************************
 * Check the status of the Hardware Receive buffer (if not empty "data to read", output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_FLAG(_uart_handle_, UART_FLAG_RXNE) != 0 )
        return (1);
    else
        return (0);

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    return (1);                         // Return a positive output, such that any downstream will
                                        // continue as if entry is set

#endif
}

void UARTPeriph::clearComptChk(void) {
/**************************************************************************************************
 * Clear the Transmission complete flag
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    __HAL_UART_CLEAR_FLAG(_uart_handle_, UART_FLAG_TC); // Clear status register

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    __HAL_UART_CLEAR_FLAG(_uart_handle_, UART_CLEAR_TCF); // Clear status register

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Do nothing...

#endif
}

uint8_t UARTPeriph::transmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Empty Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_TXE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_TXE) != 0 )
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

uint8_t UARTPeriph::transmitComptITChk(void) {
/**************************************************************************************************
 * Check to see whether the Transmit Complete Interrupt has been enabled (if enabled, output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_TC) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_TC) != 0 )
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

uint8_t UARTPeriph::receiveToReadITChk(void) {
/**************************************************************************************************
 * Check to see whether the Receive Buffer full interrupt has been enabled (if enabled,
 * output = 1)
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_RXNE) != 0 )
        return (1);
    else
        return (0);

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if ( __HAL_UART_GET_IT_SOURCE(_uart_handle_, UART_IT_RXNE) != 0 )
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

UARTPeriph::Form UARTPeriph::genericForm(uint8_t *data, uint16_t size,
                                         volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Generate a UARTForm request, based upon the generic information provided as input.
 *************************************************************************************************/
    Form request_form = { 0 };                  // Generate the "Form" variable to provide as 
                                                // output

    request_form.Buff           = data;         // Populate form with input data
    request_form.size           = size;         //

    // Indications used for source functionality to get status of requested communication
    request_form.Flt            = fltReturn;    // Populate return fault flag
    request_form.Cmplt          = cmpFlag;      // Populate complete communication indication

    return (request_form);
}

uint8_t UARTPeriph::getFormWriteData(Form *RequestForm) {
/**************************************************************************************************
 * Retrieve the next data point to write to external device from the selected UART Request form
 *************************************************************************************************/
    uint8_t temp_val = 0;       // Temporary variable to store data value

    temp_val = *(RequestForm->Buff);            // Retrieve data from array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer

    return (temp_val);       // Return value outside of function
}

void UARTPeriph::putFormReadData(Form *RequestForm, uint8_t readdata) {
/**************************************************************************************************
 * Data read from the I2C external device is copied into the requested source location, as per
 * the UART Request form
 *************************************************************************************************/
    *(RequestForm->Buff)    = readdata;         // Put data into array
    RequestForm->Buff       += sizeof(uint8_t); // Increment array pointer
}

uint8_t UARTPeriph::poleSingleRead(void) {
/**************************************************************************************************
 * Will take a single byte of data from the UART peripheral, and return out of function.
 * Note that this will WAIT until there is data available to be read.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    // Ensure that the UART interface has been enabled
    if ((_uart_handle_->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(_uart_handle_);   // Enable the UART interface if not enabled

        // Check to see if there is data to be read, done by checking the Read Data Register not
        // empty flag (RXNE), if this is TRUE then there is data to be read.
    while (receiveToReadChk() != 1) {}

    return (readDR());  // Retrieve the read data, and pass out of function

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    // Ensure that the UART interface has been enabled
    if ((_uart_handle_->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(_uart_handle_); // Enable the UART interface if not enabled

        // Check to see if there is data to be read, done by checking the Read Data Register not
        // empty flag (RXNE), if this is TRUE then there is data to be read.
    while (receiveToReadChk() != 1) {}

    return (readDR());  // Retrieve the read data, and pass out of function

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    int read_back_data = -1;        // Create a variable which will contain the read contents of
                                    // the UART device
                                    // Set this to -1, as this indicates that there is no data to
                                    // be read, and need to loop until data is read
    while(read_back_data == -1) {   // Loop until get data from UART
        read_back_data = readDR();                  // Read data from UART
            // Function will time out after 10s returning -1. As want to pole until new data is
            // available, keep looping until get anything but -1
    }

    return ((uint8_t) read_back_data);  // If get to this point data has been read from UART,
                                        // therefore return read value

#endif
}

void UARTPeriph::poleSingleTransmit(uint8_t data) {
/**************************************************************************************************
 * Will take the provided data, and put onto the UART peripheral for transmission.
 * Note that this will WAIT until it is able to transmit the data.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    // Ensure that the UART interface has been enabled
    if ((_uart_handle_->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(_uart_handle_);   // Enable the UART interface if not enabled

        // Check to see if the Transmit Data Register is empty (TXE), this will be set to TRUE
        // by the hardware to indicate that the contents of the TDR register have been setup for
        // transmission. Therefore new data can be added for transmission
    while (transmitEmptyChk() == 0) {}

    writeDR(data);    // Now put the selected data onto the Data Register (DR) for transmission.


#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    // Ensure that the UART interface has been enabled
    if ((_uart_handle_->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(_uart_handle_);   // Enable the UART interface if not enabled

        // Check to see if the Transmit Data Register is empty (TXE), this will be set to TRUE
        // by the hardware to indicate that the contents of the TDR register have been setup for
        // transmission. Therefore new data can be added for transmission
    while (transmitEmptyChk() == 0) {}

    writeDR(data);      // Now put the selected data onto the Data Register (DR) for transmission.

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    writeDR(data);      // Send data via UART

#else
//=================================================================================================
// Do nothing

#endif
}

UARTPeriph::DevFlt UARTPeriph::poleTransmit(uint8_t *pData, uint16_t size) {
/**************************************************************************************************
 * An extension of the Single Transmit function, this allows for an array of data to be set via
 * UART.
 * Again it will wait until it can transmit, and all data has been transmitted before exiting.
 * > However if there is no data, or the size is zero, it will return a fault.
 *************************************************************************************************/

    if (pData == __null || size == 0)       // If no data has been requested to be set
        return (flt = DevFlt::kData_Error); // return error with DATA (also update fault state)

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    while (size > 0) {                      // Whilst there is data to be transferred
        poleSingleTransmit(*pData);   // Transmit the single point of data
        pData += sizeof(uint8_t);           // Increment pointer by the size of the data to be
                                            // transmitted
        size--;                             // Decrement the size
    }

    // Wait for final data point to have completed transmission before continuing
    while (transmitComptChk() == 0) {}

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    while (size > 0) {                      // Whilst there is data to be transferred
        poleSingleTransmit(*pData);         // Transmit the single point of data
        pData += sizeof(uint8_t);           // Increment pointer by the size of the data to be
                                            // transmitted
        size--;                             // Decrement the size
    }

#else
//=================================================================================================
// Do nothing

#endif

    return (flt = DevFlt::kNone);   // If have made it to this point, then there is no issues
                                    // also update fault state

}

void UARTPeriph::configTransmtIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_TXE);   // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_TXE);  // Then disable interrupt
    }

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_TXE);   // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_TXE);  // Then disable interrupt
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

void UARTPeriph::configTransCmIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Complete interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_TC);    // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_TC);   // Then disable interrupt
    }

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_TC);    // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_TC);   // Then disable interrupt
    }

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(ktransmit_complete);
        // Enable the pseudo Transmit complete bit - via the "Pseudo interrupt" register
    }
    else {                                                      // If request is to disable
        pseudoRegisterClear(ktransmit_complete);
        // Disable the pseudo Transmit complete bit - via the "Pseudo interrupt" register
    }

#endif
}

void UARTPeriph::configReceiveIT(InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive buffer full interrupt.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_RXNE);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_RXNE); // Then disable interrupt
    }

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_UART_ENABLE_IT(_uart_handle_, UART_IT_RXNE);  // Then enable the interrupt
    }
    else {                                                  // If request is to disable
        __HAL_UART_DISABLE_IT(_uart_handle_, UART_IT_RXNE); // Then disable interrupt
    }

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        pseudoRegisterSet(kread_data_register_not_empty);
        // Enable the pseudo Receive bit - via the "Pseudo interrupt" register
    }
    else {                                                      // If request is to disable
        pseudoRegisterClear(kread_data_register_not_empty);
        // Disable the pseudo Receive bit - via the "Pseudo interrupt" register
    }

#endif
}

void UARTPeriph::intWrtePacket(uint8_t *wData, uint16_t size,
                               volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function will generate a new form for the write UART communication
 *************************************************************************************************/
    Form request_form = genericForm(wData, size, fltReturn, cmpFlag);

    _form_wrte_q_.inputWrite(request_form);
    // Add to queue

    // Trigger interrupt(s)
    startInterrupt();
}

void UARTPeriph::intReadPacket(uint8_t *rData, uint16_t size,
                               volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Function will generate a new form for the read UART communication
 *************************************************************************************************/
    Form request_form = genericForm(rData, size, fltReturn, cmpFlag);

    _form_read_q_.inputWrite(request_form);
    // Add to queue

    // Trigger interrupt(s)
    startInterrupt();
}

void UARTPeriph::startInterrupt(void) {
/**************************************************************************************************
 * Function will be called to start off any new UART communication (read/write) if there is
 * anything within either of the queues, and the bus is free.
 *************************************************************************************************/
    // Write Communication setup
    if ( (wrte_comm_state == CommLock::kFree) && (_form_wrte_q_.state() != kGenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        _form_wrte_q_.outputRead( &(_cur_wrte_form_) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_wrte_form_.Flt) != DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_wrte_q_.state() == kGenBuffer_Empty ) {  // If buffer is empty break out
                //Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_wrte_q_.outputRead( &(_cur_wrte_form_) );     // Capture form request
        }

        wrte_comm_state = CommLock::kCommunicating; // Lock UART bus

        //Enable();

        _cur_wrte_count_  = _cur_wrte_form_.size;

        configTransmtIT(InterState::kIT_Enable);    // Then enable Transmit Empty buffer interrupt
    }
    else if ( (wrte_comm_state == CommLock::kFree) &&
              (_form_wrte_q_.state() == kGenBuffer_Empty) ) {
        //Disable();
    }

    // Read Communication setup
    if ( (read_comm_state == CommLock::kFree) && (_form_read_q_.state() != kGenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        _form_read_q_.outputRead( &(_cur_read_form_) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_read_form_.Flt) != DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_read_q_.state() == kGenBuffer_Empty ) {  // If buffer is empty, break out
                //Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_read_q_.outputRead( &(_cur_read_form_) );     // Capture form request
        }

        read_comm_state = CommLock::kCommunicating; // Lock UART bus

        //Enable();

        _cur_read_count_  = _cur_read_form_.size;

        configReceiveIT(InterState::kIT_Enable);    // Then enable Receive buffer full interrupt
    }
    else if ( (read_comm_state == CommLock::kFree) &&
              (_form_read_q_.state() == kGenBuffer_Empty) ) {
        //Disable();
    }
}

void UARTPeriph::intWrteFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form for writing, to indicate how much data has been completed.
 * Indicates that the UART write Device is now free for any new communication
 * Disables the Transmit Interrupts
 *************************************************************************************************/
    *(_cur_wrte_form_.Cmplt)  += (_cur_wrte_form_.size - _cur_wrte_count_);
    // Indicate how many data points have been transfered (curCount should be 0)

    // Indicate that UART bus is now free, and disable any interrupts
    wrte_comm_state = kFree;

    configTransmtIT(InterState::kIT_Disable);   // Disable Transmit empty buffer interrupt
    configTransCmIT(InterState::kIT_Disable);   // Disable Transmit complete interrupt
}

void UARTPeriph::intReadFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form for reading, to indicate how much data has been completed.
 * Indicates that the UART read Device is now free for any new communication
 * Disables the Receive Interrupts
 *************************************************************************************************/
    *(_cur_read_form_.Cmplt)  += (_cur_read_form_.size - _cur_read_count_);
    // Indicate how many data points have been transfered (curCount should be 0)

    // Indicate that UART bus is now free, and disable any interrupts
    read_comm_state = CommLock::kFree;

    configReceiveIT(InterState::kIT_Disable);   // Disable Receive buffer full interrupt
}

void UARTPeriph::readGenBufferLock(GenBuffer<uint8_t> *ReadArray,
                                   volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Intended to allow link to a GenBuffer class, whilst also utilising the UART Form system.
 * Will in essence ensure that there is always a request to read data, and put this into the
 * GenBuffer.
 *   Will check to see if the communication for read becomes "Free" (current read request has
 *   its intended size), will request a new read - pointing to the top of the GenBuffer array, and
 *   enter the size of the array (retrieved from the GenBuffer class). Then provides top level
 *   fault status and completed flags (won't really be used).
 *
 * Finally, will update the 'input_pointer' of the GenBuffer, so as to align with the current read
 * status.
 *
 *************************************************************************************************/
    if (read_comm_state == CommLock::kFree) {       // If the UART Receive is free to be used
        *fltReturn = DevFlt::kNone;                 // Clear the linked fault flag
        *cmpFlag = 0;                               // Clear complete flag

        intReadPacket( ReadArray->pa, ReadArray->length, fltReturn, cmpFlag);
            // Request a new read back
    }

    ReadArray->input_pointer = ReadArray->length - _cur_read_count_;
    // use internal class structure - 'curReadCount' to calculate how many data points have
    // been read back
}

void UARTPeriph::handleIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UART class. Each of the supported devices needs to call this
 * function in different ways - therefore each implementation is mentioned within the coded
 * section.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UART class. Each of the supported devices needs to call this
 * function in different ways - therefore each implementation is mentioned within the coded
 * section.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    if ( (transmitComptChk() & transmitComptITChk()) == 0x01) { //
        clearComptChk();            // Clear the Transmission complete flag

        intWrteFormCmplt();         // Complete the current request form (no faults)
        startInterrupt();           // Check if any new requests remain

    }

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if ( (receiveToReadChk() & receiveToReadITChk()) == 0x01) {
        putFormReadData( &(_cur_read_form_) , readDR() );
        // Put next data point into the area requested from the UART Form
        _cur_read_count_--;         // Decrement the class global current count

        if (_cur_read_count_ == 0) {
            intReadFormCmplt();         // Complete the current request form (no faults)
            startInterrupt();           // Check if any new requests remain
        }
    }

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01) {
        writeDR (  getFormWriteData( &(_cur_wrte_form_) )  );
        // Retrieve next data point from the Request UART Form, and put onto hardware queue
        _cur_wrte_count_--;         // Decrement the class global current count

        if (_cur_wrte_count_ == 0) {
            intWrteFormCmplt();         // Complete the current request form (no faults)
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
 *  UART peripheral buffer or if data has been requested to be sent (via pseudo interrupt
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
 *  uint8_t UARTReturn;
 *  while (1) {
 *      delay(100);
 *      MyUART->handleIRQ();
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the UARTPeriph will need to be made global to allow this
 * new thread to all the "IRQHandler"
 *************************************************************************************************/
    if ( (transmitComptChk() & transmitComptITChk()) == 0x01) { //
        clearComptChk();            // Clear the Transmission complete flag

        intWrteFormCmplt();         // Complete the current request form (no faults)
        startInterrupt();           // Check if any new requests remain

    }

    int buffer_contents = 0;            // Variable to store the amount of data in UART peripheral

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if ( (receiveToReadChk() & receiveToReadITChk()) == 0x01) {
        buffer_contents = anySerDataAvil();     // Get the amount of data in UART

        while (buffer_contents > 0) {
            putFormReadData( &(_cur_read_form_) , readDR() );
            // Put next data point into the area requested from the UART Form
            _cur_read_count_--;         // Decrement the class global current count

            if (_cur_read_count_ == 0) {
                intReadFormCmplt();         // Complete the current request form (no faults)
                startInterrupt();           // Check if any new requests remain
            }

            buffer_contents--;
        }
    }

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if ( (transmitEmptyChk() & transmitEmptyITChk()) == 0x01) {

        while (_cur_wrte_count_ != 0) { // Will loop continually so long as there is data to be
                                        // transmitted.
            // Call to 'startInterrupt' will ensure that '_cur_wrte_count_' is reset to != '0' if
            // there is any other request in the system

            writeDR (  getFormWriteData( &(_cur_wrte_form_) )  );
            // Retrieve next data point from the Request UART Form, and put onto hardware queue
            _cur_wrte_count_--;         // Decrement the class global current count

            if (_cur_wrte_count_ == 0) {
                intWrteFormCmplt();         // Complete the current request form (no faults)
                startInterrupt();           // Check if any new requests remain
            }
        }
    }
#else
//=================================================================================================

#endif
}

UARTPeriph::~UARTPeriph() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    if (close(_uart_handle_) < 0)
        errorMessage("Error, unable to close USART device - %s", _device_loc_);
#else
//=================================================================================================

#endif
}

