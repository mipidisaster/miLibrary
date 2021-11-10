/**************************************************************************************************
 * @file        GPIO.cpp
 * @author      Thomas
 * @brief       Source file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_GPIO___HD               // Header for GPIO

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#include <stdio.h>                      // Standard I/O - including the error file
#include <stdarg.h>                     // Allows functions to accept an indefinite number of
                                        // arguments
#include <stdlib.h>                     // Needed for 'exit'

#include <fcntl.h>                      // Needed for GPIO port
#include <unistd.h>                     //

#include <iostream>                     // Used to allow for string manipulation
#include <fstream>                      //
#include <sstream>                      //
#include <string>                       //

// See  https://raspberry-projects.com/pi/programming-in-c/io-pins
//      --> Most useful was the link in io_speed (if want to use memory locations in future)
//              > https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access
//              > https://elinux.org/RPi_GPIO_Code_Samples#sysfs (per current design)
//          https://linux.die.net/

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
#include <stdio.h>                      // Standard I/O - including the error file
#include <stdarg.h>                     // Allows functions to accept an indefinite number of
                                        // arguments
#include <stdlib.h>                     // Needed for 'exit'

#include <iostream>                     // Used to allow for string manipulation
#include <fstream>                      //
#include <sstream>                      //
#include <string>                       //

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------

//=================================================================================================

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
GPIO::GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, Dir direction) {
/**************************************************************************************************
 * Create a GPIO class specific for the STM32F device
 * Receives the PortAddress pointer, and pin number - all comes from the cubeMX output
 * Also requires the direction of the pin - INPUT/OUTPUT
 * Not setup of the port clock or pin, covered by the cubeMX outputs
 *************************************************************************************************/
    _pin_number_      = pinnumber;      // copy data into class
    _port_address_    = PortAddress;    //
    _pin_direction_   = direction;      //
}
#else   // Raspberry Pi or Default build configuration
//=================================================================================================
GPIO::GPIO(State pinvalue, uint32_t pinnumber, Dir pindirection) {
/**************************************************************************************************
 * Create a GPIO class specific for the Raspberry Pi/Default build
 * Receives the file location of where the gpio is located - i.e. /sys/class/gpio/gpio<number>, as
 * well as the pin state (Low/High), and direction (Input/Output), and pin number again.
 *************************************************************************************************/
    _pin_number_        = pinnumber;    // copy data into class
    _pin_direction_     = pindirection; //
    _pin_value_         = pinvalue;     //

    std::ostringstream s;
    s << "gpio" << _pin_number_;
    _device_loc_ = kgpio_file_location + std::string(s.str()) + "/";
        // Outcome should be something like:
        //  "/sys/class/gpio/gpio1/" (if pinnumber was '1')

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    exportGPIO();                       // Setup GPIO to export mode
    usleep(250000);                     // 250ms delay

    setDirection(_pin_direction_);      // Configure for desired direction

    if (_pin_direction_ == Dir::kOutput) {  // If OUTPUT, set to desired starting state
        setValue(_pin_value_);
    }

#endif
}

void GPIO::piSetup(void) {
/**************************************************************************************************
 * Static function which wraps the 'wiringPi' routines within my 'GPIO' class.
 * Function is to only be called once, otherwise it will run out of file handles, so be careful
 * when using.
 * Note - this doesn't actually do anything! - kept to maintain plug'n'play with previous
 * version(s) of GPIO
 *************************************************************************************************/
    // Do nothing
}

void GPIO::errorMessage(const char *message, ...) {
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

int GPIO::write(std::string path, std::string filename, std::string value){
/**************************************************************************************************
 * Function will setup an OUTPUT filestream with the desired GPIO path, combining together the path
 * and filename.
 * Once stream is setup correctly, will drive the 'value' into the stream. Before closing down
 * stream
 *************************************************************************************************/
    std::ofstream fs;                       // OUTPUT STREAM
    fs.open((path + filename).c_str());     // Open up desired file stream
    if (!fs.is_open())
        errorMessage("Unable to write to GPIO path device: %s", (path + filename).c_str());

    fs << value;
    fs.close();
    return 0;
}

int GPIO::write(std::string path, std::string filename, int value) {
/**************************************************************************************************
 * Convenience function.
 * Same as the 'write' above, however uses a internal value instead of string.
 *************************************************************************************************/
    std::stringstream s;
    s << value;

    return write(path, filename, s.str());
}

std::string GPIO::read(std::string path, std::string filename) {
/**************************************************************************************************
 * Function will setup an INPUT filestream with the desired GPIO path, combining together the path
 * and filename.
 * Once stream is setup correctly, will drive the 'value' into the stream. Before closing down
 * stream
 *************************************************************************************************/
    std::ifstream fs;                       // INPUT STREAM
    fs.open((path + filename).c_str());     // Open up desired file stream
    if (!fs.is_open())
        errorMessage("Unable to read from GPIO path device: %s", (path + filename).c_str());

    std::string input;
    std::getline(fs, input);
    fs.close();

    return input;
}

int GPIO::exportGPIO(void) {
/**************************************************************************************************
 * Within the gpio file location, setup the desired GPIO to be in export mode (so can be used to
 * read/write/etc.)
 *************************************************************************************************/
    // Prior to exporting GPIO lets check to see if it is already open...
    std::ifstream fs;                       // INPUT STREAM
    fs.open((kgpio_file_location + "export").c_str());  // Open up desired file stream

    if (!fs.is_open()) {    // If unable to open, then not been exported yet
        fs.close();         // Close current stream

        return write(kgpio_file_location, "export", _pin_number_);
    }

    // If get here then there has already been an export of the GPIO....so lets get rid of it!
    unexportGPIO();
    return write(kgpio_file_location, "export", _pin_number_);
}

int GPIO::unexportGPIO(void) {
/**************************************************************************************************
 * Within the gpio file location, setup the desired GPIO to be in unexported (closing down the
 * GPIO for control)
 *************************************************************************************************/
    return write(kgpio_file_location, "unexport", _pin_number_);
}

int GPIO::setDirection(Dir pindirection) {
/**************************************************************************************************
 * Configure for desired direction
 *************************************************************************************************/
    if (pindirection == Dir::kInput)
        return write(_device_loc_, "direction", "in");

    else
        return write(_device_loc_, "direction", "out");

    return -1;  // If gets to here return failure!
}

#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kOutput)    // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    _port_address_->ODR  ^= _pin_number_;   // Toggle specific pin

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    if (_pin_value_ == State::kLow) // If the Pin is set at LOW
        setValue(State::kHigh);     // Update to HIGH, function also updates the "pinvalue"
    else                            // If the Pin is set at HIGH
        setValue(State::kLow);      // Update to LOW, function also updates the "pinvalue"

#else
//=================================================================================================
// As is default configuration; there is no hardware to interact with - do nothing

#endif

    return 0;
}

uint8_t GPIO::setValue(State value) {
/**************************************************************************************************
 * Set the state of the output pin as per user demand "value", ONLY if the class has been declared
 * as an OUTPUT
 * Otherwise return error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kOutput)    // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    if (value == State::kLow)           // If demand is to set it LOW
        _port_address_->BSRR     =   (uint32_t)_pin_number_ << 16U;
            // Set the corresponding RESET bit within the BSRR register (shifted up by 16bits)
    else
        _port_address_->BSRR     =   _pin_number_;
            // Set the corresponding SET bit within the BSRR register

#else       // Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    if (value == State::kLow)               // If demand for Pin is set for LOW
        write(_device_loc_, "value", "0");  // Drive pin LOW

    else
        write(_device_loc_, "value", "1");  // Drive pin High

#endif

    _pin_value_     = value;                    // Update pin value

#endif

    return 0;
}

GPIO::State GPIO::getValue() {
/**************************************************************************************************
 * Read the state of the input pin. Function will only work on INPUT pin, if pin is declared as an
 * OUTPUT it will return an error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kInput)     // Check direction of pin, if not equal to INPUT
        return State::kLow;                 // return LOW state

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

    if (( _port_address_->IDR & _pin_number_ ) != 0 )
            // Check the state of the pin, and if it is set (i.e. not equal to zero), then output
            // HIGH status
        return State::kHigh;
    else    // Otherwise output LOW status
        return State::kLow;

#else       // Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    std::string input = read(_device_loc_, "value");

    if (input == "1")   return State::kHigh;
    else                return State::kLow;

#endif

    return State::kLow;

#endif
}

GPIO::~GPIO() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
// Nothing needs to be done

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
    unexportGPIO();     // Close down GPIO connections

#endif
    // TODO Auto-generated destructor stub
}
