/**************************************************************************************************
 * @file        StepperCore.cpp
 * @author      Thomas
 * @brief       Source file for the Stepper Driver Class handle (Core control)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_StpCOREHD               // Header for Stepper (Core Header)

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

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
#include FilInd_GENBUF_TP               // Allow use of GenBuffer template class
#include FilInd_GPIO___HD               // Allow use of GPIO class

void StepperCore::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the Stepper class. To be called by all constructors.
 *************************************************************************************************/
    // Populate the Shadow Form with default parameters driving the Stepper Driver off
    _shd_form_.Dir      = GPIO::State::kLow; // Pull direction low
    _shd_form_.nRst     = GPIO::State::kLow;    // Set RESET low = Reset
    for (uint8_t i = 0; i != knumber_micro_pins; i++) {
            // Cycle through array of Microstep states and initialise GPIOs low
        _shd_form_.Micr[i] = GPIO::State::kLow;
    }
    _shd_form_.Freq     = 0;                    // Initialise the frequency to zero
    _shd_form_.StpCount = 0;                    // Initialise count to zero

    _shd_form_.cMode    = Mode::kDisabled;      // Indicate that stepper is disabled

    _shd_count_conf_.stpAmount    = 0;

    _shd_count_conf_.Pol          = CountPanel::Polarity::kUp;

    _shd_form_.CountConf  = _shd_count_conf_;

    for (uint8_t i = 0; i != kform_size; i++) {     // Loop through entries within Form array
        _form_arry_[i]  = _shd_form_;               // and set equal to shadow form (which is now
                                                    // in default state
    }

    _form_queue_.create(&_form_arry_[0], kform_size);   // Create GenBuffer class, and link to
                                                        // declared form array

    _micro_step_size_   = 0;        // Initialise the number of Microstep pins to zero

    full_revolution     = 0;        // Set the number of steps for full revolution to zero
    calc_position       = 0;        // Initialise pole Position calculation to zero
}

StepperCore::StepperCore(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA,
                         uint32_t OCChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp,
                         uint8_t McrStp, int32_t FullRev, HrdSetup Config) {
/**************************************************************************************************
 * Creates a Stepper class specific for the STM32F device.
 *  Inputs include the TIMER handle which will be used to control the stepper motor.
 *  Along with the Output Compare channel (which is the channel state shifted up by the channel)
 *  The linked DMA is also required, to provide to the Output Compare output (alias "_step_") with
 *  the required sequence to generate the STEP pulse.
 *
 *  Then all the auxillary GPIO pins - Reset, and Microstep(s) < last one needs to also include
 *                                                               the number of pins for this
 *  The number of steps per revolution of the Stepper need to be provided in the format of
 *  hardware number of steps multiplied by the smallest step size:
 *      i.e. motor has 200 poles, but driver can provide a step of 1/16
 *           "full_revolution"  = 200 x 16 = 3,200 steps
 *
 *  Lastly the Hardware Setup structure is required. This should be configured for the specific
 *  embedded device (limited to STM32 devices currently). Including the bit positions and address
 *  to manage interrupts and the DMA.
 *************************************************************************************************/
    popGenParam();                      // Populate generic class parameters

    _nreset_pin_        = nReset;       // Link the Reset pin to the class internals
    _direction_         = DIR;          // Link the Direction pin to the class internals

    _micro_step_        = MicStp;       // Link input Microstep array to class internals
    _micro_step_size_   = McrStp;       // Put number of microstep pins into array

    full_revolution   = FullRev;        // Copy Full Revolution number to class internals

    _step_                  = STEP_TIM;     // Link the STEP timer to class internals
    _dma_stp_               = STEP_DMA;     // Link timer DMA to class internals
    _output_compare_channel_= OCChannel;    // Provide channel output for "Output Compare"
                                            // (OUTPUT)
    _hardware_config_       = Config;       // Link Hardware setup to class internals

    calc_position   = 0;                    // Initialise the calculated position to 0
}

void StepperCore::setShadowGPIO(void) {
/**************************************************************************************************
 * Configure the internally linked GPIO pins (RESET, DIR, MicroStep) as per the Shadow Form
 *************************************************************************************************/
    _nreset_pin_->setValue(_shd_form_.nRst);    // Set the RESET pin to desired state
    _direction_->setValue(_shd_form_.Dir);      // Set the DIR pin to desired state

    for (uint8_t i = 0; i != _micro_step_size_; i++) {          // Loop through Micro pins
        _micro_step_[i].setValue(_shd_form_.Micr[i]);           // and set to desired value
    }

    _shd_count_conf_ = _shd_form_.CountConf;   // Bring count configuration to shadow controller
    // Set at same time as GPIO, to ensure count is synched correctly.
}

void StepperCore::forceSTOP(void) {
/**************************************************************************************************
 * Function will force the stepper to ignore current request, and clear all queued requests.
 * This will result in the stepper coming to a stop
 *************************************************************************************************/
    _form_queue_.qFlush();              // First clear the queue
    Form tmp_form    = _shd_form_;      // Create a new temporary form equal to current shadow

    // Put in a Position Mode request for a single step
    tmp_form.StpCount   = 1;                        // Single step
    tmp_form.cMode      = Mode::kPosition;          // Set mode to "Position"

    if (_shd_form_.cMode != Mode::kDisabled) {      // If motor is not disabled (yet)
        _form_queue_.inputWrite( tmp_form );        // Put new request into queue
        _shd_form_.cMode  = Mode::kDisabled;        // Force the current mode to Disabled
        //This will force interrupt to stop current movement, and read new request
    }
}

void StepperCore::newPosition(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq,
                              uint32_t PulseCount, CountPanel::Polarity Pol, uint32_t StpprPul) {
/**************************************************************************************************
 * Request a movement of the stepper motor by "PulseCount" of STEPS, at a frequency of "Freq".
 * Defines the desired state of the Direction pin for movement, along with the Microstep pins
 *      Input for Microstep is provided as a unsigned integer, where the 1st bit is the 1st GPIO
 *      pin provided at class construction
 *      Any entries outside of the number of pins provided will be ignored.
 *
 *  Pol and StpperPul are parameters used for the class internal calculation for pole position.
 *  Pol      - Defines the polarity (Count UP, or DOWN)
 *  StpprPul - Defines the count per STEP
 *************************************************************************************************/
    Form tmp_form;                                  // Temporary form prior to entry into buffer

    tmp_form.Dir     = DIR;                         // Copy across the desired Direction state
    tmp_form.nRst    = GPIO::State::kHigh;          // Set RESET to "HIGH" - No RESET

    for (uint8_t i = 0; i != knumber_micro_pins; i++) {     // Loop through the Microstep pins
                                                            // (pre-processor size)
        if (i >= _micro_step_size_) {                   // If loop is greater than defined number
                                                        // of GPIO pins
            tmp_form.Micr[i]     = GPIO::State::kLow;   // Default to "LOW"
        }
        else {                                          // If within defined number of GPIOs
            if (MicroStp & 1)                           // If 1st bit is set to "1"
                tmp_form.Micr[i] = GPIO::State::kHigh;  // Set state to "HIGH"
            else                                        // OTHERWISE
                tmp_form.Micr[i] = GPIO::State::kLow;   // Set state to "LOW"

            MicroStp >>= 1;     // Shift down by 1 bit
    } }

    tmp_form.CountConf.Pol           = Pol;         // Copy across both Polarity and Count per
    tmp_form.CountConf.stpAmount     = StpprPul;    // step into Form Counter Configuration

    if (Freq < kmax_frequency) { Freq = kmax_frequency; };
        // If the desired speed is faster than can be support, saturate

    tmp_form.Freq        = Freq;                    // Copy across Frequency into form
    tmp_form.StpCount    = PulseCount;              // Copy across number of pulses into form
    tmp_form.cMode       = Mode::kPosition;         // Set mode to "Position"

    _form_queue_.inputWrite( tmp_form );    // Put temporary form into queue
    startInterrupt();                       // Call interrupt setup function
}

void StepperCore::newVelocity(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq,
                              CountPanel::Polarity Pol, uint32_t StpprPul) {
/**************************************************************************************************
 * Request a continual movement of the stepper motor at the rate of "Freq".
 * Define the desired state of the Direction pin for movement, along with the Microstep pins
 *      Input for Microstep is provided as a unsigned integer, where the 1st bit is the 1st GPIO
 *      pin provided at class construction
 *      Any entries outside of the number of pins provided will be ignored.
 *
 *  Pol and StpperPul are parameters used for the class internal calculation for pole position.
 *  Pol     - Defines the polarity (Count UP, or DOWN)
 *  StppPul - Defines the count per STEP
 *************************************************************************************************/
    Form tmp_form;                                  // Temporary form prior to entry into buffer

    tmp_form.Dir     = DIR;                         // Copy across the desired Direction state
    tmp_form.nRst    = GPIO::State::kHigh;          // Set RESET to "HIGH" - No RESET

    for (uint8_t i = 0; i != knumber_micro_pins; i++) {     // Loop through the Microstep pins
                                                            // (pre-processor size)
        if (i >= _micro_step_size_) {                   // If loop is greater than defined number
                                                        // of GPIO pins
            tmp_form.Micr[i]     = GPIO::State::kLow;   // Default to "LOW"
        }
        else {                                          // If within defined number of GPIOs
            if (MicroStp & 1)                           // If 1st bit is set to "1"
                tmp_form.Micr[i] = GPIO::State::kHigh;  // Set state to "HIGH"
            else                                        // OTHERWISE
                tmp_form.Micr[i] = GPIO::State::kLow;   // Set state to "LOW"

            MicroStp >>= 1;     // Shift down by 1 bit
    } }

    tmp_form.CountConf.Pol           = Pol;         // Copy across both Polarity and Count per
    tmp_form.CountConf.stpAmount     = StpprPul;    // step into Form Counter Configuration

    if (Freq < kmax_frequency) { Freq = kmax_frequency; };
        // If the desired speed is faster than can be support, saturate

    tmp_form.Freq        = Freq;                    // Copy across Frequency into form
    tmp_form.StpCount    = 0;                       // As in "Velocity" mode no step count is
                                                    // required
    tmp_form.cMode       = Mode::kVelocity;         // Set mode to "Velocity"

    _form_queue_.inputWrite( tmp_form );    // Put temporary form into queue
    startInterrupt();                       // Call interrupt setup function
}

void StepperCore::updateModelPosition(uint8_t number_pulses) {
/**************************************************************************************************
 * Calculate the 'modelled' position of the stepper motor, based upon the expected step amount
 * per pulse (from the shadow form)
 *************************************************************************************************/

    if (_shd_count_conf_.Pol == CountPanel::Polarity::kUp) {
    // If current Count Configuration is count "UP"
        calc_position = (calc_position + (_shd_count_conf_.stpAmount * number_pulses))
                        % full_revolution;
            // Then add the specified count per STEP to the "calcPos". This is then limited to
            // the maximum number of steps per revolution
    }
    else {  // OTHERWISE polarity is "DOWN"
        calc_position = ( (calc_position - (_shd_count_conf_.stpAmount * number_pulses))
                          + full_revolution )
                          % full_revolution;
            // Then subtract the specified count per STEP to the "calcPos". This is then
            // limited to the maximum number of steps per revolution
    }
}

uint8_t  StepperCore::getSelectedMicroStep(void) {
/**************************************************************************************************
 * Return the current state of the microstep selected.
 *************************************************************************************************/
    uint8_t current_microstep = 0;

    for (uint8_t i = 0; i != knumber_micro_pins; i++) {
        if (i >= _micro_step_size_) {
            // Do nothing as this will be zero
        }
        else {
            if (_shd_form_.Micr[i] == GPIO::State::kHigh) {
                current_microstep |= 1;
            }
        }

        current_microstep <<= 1;    // Shift up the state
    }

    return (current_microstep);
}

uint8_t  StepperCore::getSelectedDirection(void) {
/**************************************************************************************************
 * Return the current state of the direction.
 *************************************************************************************************/
    uint8_t current_direction = 0;

    if (_shd_form_.Dir == GPIO::State::kHigh) {
        current_direction = 1;
    }

    return (current_direction);
}

uint16_t StepperCore::getSelectedFrequency(void) {
/**************************************************************************************************
 * Return the current state of the frequency.
 *************************************************************************************************/
    return (_shd_form_.Freq);
}

StepperCore::Mode StepperCore::getCurrentMode(void) {
/**************************************************************************************************
 * Return the current mode
 *************************************************************************************************/
    return (_shd_form_.cMode);
}

void StepperCore::directAccessReset(uint8_t value) {
/**************************************************************************************************
 * Allow for direct access to the GPIOs which are provided as pointers within class construction
 * >> NOTE <<
 *   This update will happen immediately, and will therefore cause errors with the calculated
 *   position if done out of sync of the STEP pulse
 *************************************************************************************************/
    if (value != 0) {
        _nreset_pin_->setValue(GPIO::State::kHigh);
    }
    else {
        _nreset_pin_->setValue(GPIO::State::kLow);
    }
}

void StepperCore::directAccessMiroStep(uint8_t MicroStep) {
/**************************************************************************************************
 * Allow for direct access to the GPIOs which are provided as pointers within class construction
 * >> NOTE <<
 *   This update will happen immediately, and will therefore cause errors with the calculated
 *   position if done out of sync of the STEP pulse
 *************************************************************************************************/
    for (uint8_t i = 0; i != knumber_micro_pins; i++) {     // Loop through the Microstep pins
                                                            // (pre-processor size)
        if (i >= _micro_step_size_) {                   // If loop is greater than defined number
                                                        // of GPIO pins
            _micro_step_[i].setValue(GPIO::State::kLow);// Set to LOW
        }
        else {                                          // If within defined number of GPIOs
            if (MicroStep & 1)                          // If 1st bit is set to "1"
                _micro_step_[i].setValue(GPIO::State::kHigh);
            else
                _micro_step_[i].setValue(GPIO::State::kLow);

            MicroStep >>= 1;     // Shift down by 1 bit
    } }
}

void StepperCore::directAccessDirection(uint8_t value) {
/**************************************************************************************************
 * Allow for direct access to the GPIOs which are provided as pointers within class construction
 * >> NOTE <<
 *   This update will happen immediately, and will therefore cause errors with the calculated
 *   position if done out of sync of the STEP pulse
 *************************************************************************************************/
    if (value != 0) {
        _direction_->setValue(GPIO::State::kHigh);
    }
    else {
        _direction_->setValue(GPIO::State::kLow);
    }
}

StepperCore::~StepperCore()
{
    // TODO Auto-generated destructor stub
}

