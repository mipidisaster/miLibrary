/**************************************************************************************************
 * @file        StepperDMA.cpp
 * @author      Thomas
 * @brief       Source file for the Stepper Driver Class handle (DMA Interrupt)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_StpDMA_HD               // Header for Stepper (DMA)

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
#include FilInd_StpCOREHD               // Header for Stepper (Core Header)
#include FilInd_DMAPe__HD               // Allow use of the DMA class

StepperDMA::StepperDMA(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA,
                       uint32_t OCChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                       int32_t FullRev, HrdSetup Config)
/**************************************************************************************************
 * The basic construction of "StepperTIM" will make use of the "StepperCore" constructor. The
 * specifics required for DMA control is:
 *   Populate the _shd_profile_ array with default values (0)
 *   Disable the DMA system, and clear flags in the TIM
 *      All this will be enabled when a movement request is made
 *************************************************************************************************/
: StepperCore(STEP_TIM, STEP_DMA, OCChannel, nReset, DIR, MicStp, McrStp, FullRev, Config) {
    // Call the "StepperCore" constructor, which will connect all the CORE functionality required

    for (uint8_t i = 0; i != kprofile_array_pulse_depth * 2; i++) {
        _shd_profile_[i]    = 0x0000;
    }
        // Reset pulse shadow profile, to zero. This will be calculated correctly when movement
        // demands are made

    _pulse_run_     = 0;

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

void StepperDMA::disableMotor(void) {
/**************************************************************************************************
 * Function will put the Stepper class into an idle state - "Disabled", this mode is entered if
 * there is a new "Position" request, or the motor has completed a "Position" request.
 *
 * This will disable the interrupts, the DMA and switch off the Output Compare channel
 *
 *************************************************************************************************/
    _shd_form_.cMode     = Mode::kDisabled;
    configDMACmpltTransmtIT(_dma_stp_, DMAInterState::kIT_Disable);
    configDMAErrorIT(_dma_stp_, DMAInterState::kIT_Disable);
    disableDMA(_dma_stp_);

    _step_->Instance->CCER &= ~(_output_compare_channel_);
}

void StepperDMA::updatePulseDMA(uint16_t prevposition) {
/**************************************************************************************************
 * Function will calculate what the next pulse position needs to be so as to get the desired
 * frequency of PULSE(s).
 *   Note, functions 'newPosition'/'newVelocity' will already limit the maximum pulse speed, so
 *   this doesn't need to be done here.
 *************************************************************************************************/
    _pulse_run_ = kprofile_array_pulse_depth;   // Initialise this with the default size of the
                                                // _shd_profile_

    // Determine if in position mode, and if so see how many pulses are remaining. If this is less
    // then the complete size of the _shd_profile_, then only request the DMA to read to the
    // remaining pulses within the array
    if ( (_shd_form_.StpCount < kprofile_array_pulse_depth) &&
         (_shd_form_.cMode == Mode::kPosition) )
    {
        _pulse_run_ = (uint8_t) _shd_form_.StpCount;
    }

    // Cycle through the _shd_profile_ to the required number of pulses, and calculate the count
    // values for each of the next pulses.
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

void StepperDMA::startInterrupt(void) {
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

void StepperDMA::handleIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the Stepper DMA.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, the internals of this class will then manage the enabling/disabling of these
 * interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 *
 * Interrupt will only do "basic" things normally; i.e. count pulses, calculate the next DMA array
 * values. Once the number of requested steps has been achieved the main parts of this function
 * are enabled:
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
 *************************************************************************************************/
    if (globalDMAChk(_dma_stp_) != 0) {
        // Check to see if any of the connected DMA interrupts have been triggered (global
        // interrupt)
        // This function currently only supports the Complete Transmit or Error interrupts, the
        // half-transmit interrupt does nothing.

        if ( (transmitDMAErrorITChk(_dma_stp_) != 0) && (transmitDMAErrorChk(_dma_stp_) != 0) ) {
            // No need to clear the interrupt, as this is managed at the end, via clearing the
            // global flag(s)
            disableMotor();     // Disable the motor for now
        }

        if ( (comptDMATransmitITChk(_dma_stp_) != 0) && (comptDMATransmitChk(_dma_stp_) != 0) ) {
            // No need to clear the interrupt, as this is managed at the end, via clearing the
            // global flag(s)
            if (_shd_form_.StpCount != 0) {         // So long as the Shadow step count is not zero
                _shd_form_.StpCount -= _pulse_run_; // Decrement the number of pulses per DMA cycle

                updatePulseDMA(_shd_profile_[(_pulse_run_ * 2) - 1]);

            }
            updateModelPosition(  _pulse_run_  );  // Update the modelled position
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

StepperDMA::~StepperDMA()
{
    // TODO Auto-generated destructor stub
}

