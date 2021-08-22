/**************************************************************************************************
 * @file        Stepper.cpp
 * @author      Thomas
 * @brief       Source file for the Stepper Driver Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_Stppr__HD               // Header for Stepper

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
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
#include FilInd_DMAPe__HD               // Allow use of the DMA class

void Stepper::popGenParam(void) {
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

    for (uint8_t i = 0; i != kprofile_array_pulse_depth * 2; i++) {
        _shd_profile_[i]    = 0x0000;
    }
        // Reset pulse shadow profile, to zero. This will be calculated correctly when movement
        // demands are made

    _form_queue_.create(&_form_arry_[0], kform_size);

    _micro_step_size_   = 0;        // Initialise the number of Microstep pins to zero

    full_revolution     = 0;        // Set the number of steps for full revolution to zero
    calc_position       = 0;        // Initialise pole Position calculation to zero
}

Stepper::Stepper(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                 GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                 int32_t FullRev, HrdSetup Config) {
/**************************************************************************************************
 * Creates a Stepper class specific for the STM32F device.
 *  Inputs include the TIMER handle which will be used primarily to control the stepper motor
 *   >>> Doesn't support another other outputs as of yet...  <<<
 *  Along with the Output Compare channel (which is the channel state shifted up by the channel)
 *        + channel for the output compare interrupt - used to count number of steps done
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

    // Ensure that the DMA is disabled for now, and that the interrupt flag is off
    disableDMA(_dma_stp_);              // Ensure that DMA is disabled for initial setup
    __HAL_TIM_CLEAR_FLAG(_step_, _hardware_config_.PulsSRBit);  // Clear the STEP pulse
                                                                // generator bit

    // Bits cleared such that interrupts are not triggered.
    // Now enable the TIMER - the outputs and interrupts have not been enabled yet. This is
    // intentional, as do not want to have the outputs being driven during initialisation.
    if (IS_TIM_BREAK_INSTANCE(_step_->Instance) != RESET) {
        __HAL_TIM_MOE_ENABLE(_step_);
    }

    __HAL_TIM_ENABLE(_step_);   // Enable the counter if not already on
}

void Stepper::setShadowGPIO(void) {
/**************************************************************************************************
 * Configure the internally linked GPIO pins (RESET, DIR, MicroStep) as per the Shadow Form
 *************************************************************************************************/
    _nreset_pin_->setValue(_shd_form_.nRst);  	// Set the RESET pin to desired state
    _direction_->setValue(_shd_form_.Dir);   	// Set the DIR pin to desired state

    for (uint8_t i = 0; i != _micro_step_size_; i++) {     		// Loop through Micro pins
        _micro_step_[i].setValue(_shd_form_.Micr[i]);         	// and set to desired value
    }

    _shd_count_conf_ = _shd_form_.CountConf;   // Bring count configuration to shadow controller
    // Set at same time as GPIO, to ensure count is synched correctly.
}

void Stepper::disableMotor(void) {
/**************************************************************************************************
 * Function will put the Stepper class into an idle state - "Disabled", this mode is entered if
 * there is a new "Position" request, or the motor has completed a "Position" request.
 *
 * This will disable the interrupts, the DMA and switch off the Output Compare channel
 *
 *************************************************************************************************/
    _shd_form_.cMode     = Stepper::kDisabled;
    configDMACmpltTransmtIT(_dma_stp_, DMAInterState::kIT_Disable);
    configDMAErrorIT(_dma_stp_, DMAInterState::kIT_Disable);
    disableDMA(_dma_stp_);

    _step_->Instance->CCER &= ~(_output_compare_channel_);
}

void Stepper::updatePulseDMA(uint16_t prevposition) {
/**************************************************************************************************
 * Function will calculate what he next pulse position needs to be so as to get the desired
 * frequency of PULSE.
 *   Note, functions 'newPosition'/'newVelocity' will already limit the maximum pulse speed, so
 *   this doesn't need to be done here.
 *************************************************************************************************/
    _pulse_run_ = kprofile_array_pulse_depth;

    if ( (_shd_form_.StpCount < kprofile_array_pulse_depth) &&
         (_shd_form_.cMode == Mode::kPosition) )
    {
        _pulse_run_ = (uint8_t) _shd_form_.StpCount;
    }

    for (uint8_t i = 0; i != _pulse_run_*2; i = i + 2) {
        if (i == 0) {
            _shd_profile_[0] = (prevposition + _shd_form_.Freq) % kcount_limit;
            _shd_profile_[1] = (_shd_profile_[0] + kdefault_profile_width) % kcount_limit;
        }
        else {
            _shd_profile_[i]     = (_shd_profile_[i-2] + _shd_form_.Freq)
                                           % kcount_limit;
            _shd_profile_[i + 1] = (_shd_profile_[i] + kdefault_profile_width)
                                           % kcount_limit;
        }
    }

    disableDMA(_dma_stp_);
    __HAL_TIM_DISABLE_DMA(_step_, _hardware_config_.EnbTIMDMA);
    popDMARegisters(_dma_stp_,
                    (uint32_t)&_shd_profile_[0],
                    _hardware_config_.PulseDMAAddress,
                    _pulse_run_ * 2);
    // Link DMA to the internal Shadow Profile, use Hardware setup DMA Address to link to correct
    // Timer hardware register (Memory to Peripheral setup is required)
    //                         ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    __HAL_TIM_ENABLE_DMA(_step_, _hardware_config_.EnbTIMDMA);
    enableDMA(_dma_stp_);

    /* Dis-connect the DMA from the selected TIMER compare channel, re-apply the DMA
     * Memory->Peripheral connection, and the re-connect the DMA to TIMER.
     * Configure the DMA interrupts - Transmit complete, and error
     *
     * Enable DMA
     */

    _step_->Instance->EGR = ( _hardware_config_.PulsEGRBit );
    // Trigger an update of the Compare Channel (Pulse Generator - "PulsEGRBit"), so that DMA
    // has read the new pulse position.

    __HAL_TIM_CLEAR_FLAG(_step_, _hardware_config_.PulsSRBit);
        // Clear the STEP Pulse generator bit
}

void Stepper::startInterrupt(void) {
/**************************************************************************************************
 * Function checks state of the class mode, and as to whether there is any new movement requests
 * within the buffer.
 * if the mode is "Disabled", and a new request exists - then function will kick off the desired
 * stepper movement.
 *
 * If this condition is not set, then function exits.
 *************************************************************************************************/
    if ( (_shd_form_.cMode == Mode::kDisabled) &&
         (_form_queue_.state() != kGenBuffer_Empty) ) {
        _form_queue_.outputRead( &_shd_form_ ); // Retrieve next request and put into shadow form
        setShadowGPIO();                        // Setup GPIOs are per active form

        updatePulseDMA(__HAL_TIM_GET_COUNTER(_step_));
        configDMACmpltTransmtIT(_dma_stp_, DMAInterState::kIT_Enable);
        configDMAErrorIT(_dma_stp_, DMAInterState::kIT_Enable);

        _step_->Instance->CCER  |= _output_compare_channel_;
            // Enable STEP pulse Output Compare
    }
}

void Stepper::forceSTOP(void) {
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

void Stepper::newPosition(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq, uint32_t PulseCount,
                          CountPanel::Polarity Pol, uint32_t StpprPul) {
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

void Stepper::newVelocity(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq,
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

void Stepper::updateModelPosition(void) {
/**************************************************************************************************
 * Calculate the 'modelled' poisition of the stepper motor, based upon the expected step amount
 * per pulse (from the shadow form)
 *************************************************************************************************/

    if (_shd_count_conf_.Pol == CountPanel::Polarity::kUp) {
    // If current Count Configuration is count "UP"
        calc_position = (calc_position + (_shd_count_conf_.stpAmount *_pulse_run_))
                        % full_revolution;
            // Then add the specified count per STEP to the "calcPos". This is then limited to
            // the maximum number of steps per revolution
    }
    else {  // OTHERWISE polarity is "DOWN"
        calc_position = ( (calc_position - (_shd_count_conf_.stpAmount *_pulse_run_))
                          + full_revolution )
                          % full_revolution;
            // Then subtract the specified count per STEP to the "calcPos". This is then
            // limited to the maximum number of steps per revolution
    }
}

void Stepper::handleTIMxUpIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the Stepper Timer update event.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, the internals of this class will then manage the enabling/disabling of these
 * interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 *
 * This function is only expecting to be triggered for the Timer update event.
 *
 * Interrupt will only run if the ShadowForm step count is equal to zero. Otherwise interrupt will
 * exit (whilst clearing the interrupt flag).
 * If the count is equal to zero, then:
 *      If current Mode is "Velocity", then function assumes that the GPIOs will have been
 *                                     configured correctly for live request (as per
 *                                     'startInterrupt').
 *                                     If there is no new movement requests then interrupt exits.
 *                                     If there is a new movement request so long as its
 *                                     "Velocity", then new "Velocity" is selected (GPIO and
 *                                     frequency updated).
 *                                     If new movement is not "Velocity", then interrupt disables
 *                                     all interrupts, and outputs, mode is then put into
 *                                     "Disabled".
 *                                     Function "InterruptSetup" is then called to check for any
 *                                     other movement requests.
 *
 *      If current Mode is "Position", then interrupt disables all interrupts, and outputs,
 *                                     mode is then put into "Disabled".
 *                                     Function "InterruptSetup" is then called to check for any
 *                                     other movement requests.
 *
 *      No other interrupts are currently supported. However function will not run if the
 *      interrupt was not caused by a 'TIM_IT_UPDATE' event
 *************************************************************************************************/
    if (globalDMAChk(_dma_stp_) != 0) {
        if ( (transmitDMAErrorITChk(_dma_stp_) != 0) && (transmitDMAErrorChk(_dma_stp_) != 0) ) {
            clearTransmitDMAErrorFlg(_dma_stp_);
                                // Clear the interrupt flag
            disableMotor();     // Disable the motor for now
        }

        if ( (comptDMATransmitITChk(_dma_stp_) != 0) && (comptDMATransmitChk(_dma_stp_) != 0) ) {
            clearComptDMATransmitFlg(_dma_stp_);
                // Clear the interrupt flag
            if (_shd_form_.StpCount != 0) {         // So long as the Shadow step count is not zero
                _shd_form_.StpCount -= _pulse_run_; // Decrement the number of pulses per DMA cycle
               // _shd_form_.StpCount--;              // subtract by 1

                updatePulseDMA(_shd_profile_[(_pulse_run_ * 2) - 1]);

            }
            updateModelPosition();  // Update the modelled position
//=================================================================================================
            if (_shd_form_.StpCount == 0) {
                // Enter condition only when the desired number of pulses has been achieved
                // (Position mode), or every time if in Velocity mode.
                if (_shd_form_.cMode == Mode::kVelocity) {
                    if (_form_queue_.state() != kGenBuffer_Empty) {
                    // Check for any new movement requests
                        uint16_t prevpoint = _form_queue_.output_pointer;
                            // Capture current position (so as to rewind queue)
                        _form_queue_.outputRead( &_shd_form_ );   // Read new data

                        if (_shd_form_.cMode != Mode::kVelocity) {
                            // If new movement demand is anything other than 'Velocity' mode...
                            disableMotor();     // Disable motor
                            _form_queue_.output_pointer      = prevpoint;
                                // Rewind Form Queue (as has just been read)

                            startInterrupt();     // Re-check for any other move requests
                        }
                        else {  // Otherwise mode is Velocity, so update the GPIOs as per new shadow
                                // setup, and load new frequency.
                            updatePulseDMA(_shd_profile_[(_pulse_run_ * 2) - 1]);
                            setShadowGPIO();
                    } }
                    // If no change in movement is required, then recalculate DMA pulse
                    else {
                        updatePulseDMA(_shd_profile_[(_pulse_run_ * 2) - 1]);
                    }
                }
                else {
                // If mode is not velocity - then disable everything, and check for any new requests.
                    disableMotor();

                    startInterrupt();   // Check for any other move requests
                }
            }
        }

        // Ensure that all interrupts flags for this DMA are disabled
        clearGlobalDMAFlg(_dma_stp_);

    }
}

Stepper::~Stepper()
{
    // TODO Auto-generated destructor stub
}

