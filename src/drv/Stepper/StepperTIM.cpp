/**************************************************************************************************
 * @file        StepperTIM.cpp
 * @author      Thomas
 * @brief       Source file for the Stepper Driver Class handle (TIM Interrupt)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_StpTIM_HD               // Header for Stepper (TIM)

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

StepperTIM::StepperTIM(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA,
                       uint32_t OCChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                       int32_t FullRev, HrdSetup Config)
/**************************************************************************************************
 * The basic construction of "StepperTIM" will make use of the "StepperCore" constructor. The
 * specifics required for TIM control is:
 *   Setting up of the _shd_profile_ array with the values for the pulse to occur
 *   Linking and Enabling the DMA with the _shd_profile_ - and force an initial read ready for any
 *   movement requests.
 *   Clear all flags used within the Timer
 *   Then finally enabling the interrupts for the Update Event
 *************************************************************************************************/
: StepperCore(STEP_TIM, STEP_DMA, OCChannel, nReset, DIR, MicStp, McrStp, FullRev, Config) {
    // Call the "StepperCore" constructor, which will connect all the CORE functionality required

    _shd_profile_[0]    = kdefault_profile_delay;   // Put into profile delay for rising edge
    _shd_profile_[1]    = kdefault_profile_delay + kdefault_profile_width;
        // Then calculate the time (in counts) for the width of the STEP pulse

    // Configure Timer hardware for use with this Stepper class:
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Configure Timer and Linked DMA
    disableDMA(_dma_stp_);              // Ensure that DMA is disabled for initial setup
    popDMARegisters(_dma_stp_,
                    (uint32_t)&_shd_profile_[0],
                    _hardware_config_.PulseDMAAddress,
                    2);
    // Link DMA to the internal Shadow Profile, use Hardware setup DMA Address to link to correct
    // Timer hardware register (Memory to Peripheral setup is required)
    //                          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DMA also needs to be configured as a circular buffer
    //                                      ~~~~~~~~~~~~~~~

    __HAL_TIM_ENABLE_DMA(_step_, _hardware_config_.EnbTIMDMA);
        // Enable the link between Timer and DMA
    enableDMA(_dma_stp_);           // Then enable the DMA

    __HAL_TIM_SET_AUTORELOAD(_step_, 0);    // Ensure that the frequency is set to zero

    // Now trigger an update of the TIMER hardware shadow registers (by driving the EGR bits True)
    //  This needs to be done with the DMA enabled, such that when the first threshold is achieved
    //  DMA will then update the Output Compare threshold with the next - completing the PULSE

    _step_->Instance->EGR =
            ( TIM_EGR_UG | _hardware_config_.PulsEGRBit );
    // Trigger an update of the Top level counter (EGR_UR), as well as the Pulse Generator
    // "PulsEGRBit" - to get DMA 1st entry copied, as well as the Counter Interrupt

    __HAL_TIM_CLEAR_FLAG(_step_, TIM_FLAG_UPDATE);              // Clear the update bit
    __HAL_TIM_CLEAR_FLAG(_step_, _hardware_config_.PulsSRBit);  // Clear the STEP pulse
                                                                // generator bit

    // Bits cleared such that interrupts are not triggered.
    // Now enable the TIMER - the outputs and interrupts have not been enabled yet. This is
    // intentional, as do not want to have the outputs being driven during initialisation.
    if (IS_TIM_BREAK_INSTANCE(_step_->Instance) != RESET) {
        __HAL_TIM_MOE_ENABLE(_step_);
    }

    __HAL_TIM_ENABLE(_step_);
}

void StepperTIM::disableMotor(void) {
/**************************************************************************************************
 * Function will put the Stepper class into an idle state - "Disabled", this mode is entered if
 * there is a new "Position" request, or the motor has completed a "Position" request.
 *
 * This will disable the interrupts, the DMA and switch off the Output Compare channel
 *
 *************************************************************************************************/
    _shd_form_.cMode     = Mode::kDisabled;
    __HAL_TIM_DISABLE_IT(_step_, TIM_IT_UPDATE);
        // Disable Stepper Controller, and Step Count interrupt(s), and mark
        // the mode of the device as "kDisabled"

    _step_->Instance->CCER &= ~(_output_compare_channel_);
            // Clear the output enabling pin for the STEP pulse
}

void StepperTIM::startInterrupt(void) {
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

        __HAL_TIM_SET_AUTORELOAD(_step_, _shd_form_.Freq);   // Update with the requested frequency
        // Now go about updating the timer shadow registers, assumes that the interrupts and
        // outputs are already switched off, so now:
        //  1st Disable link to DMA (do not want timer update to trigger a DMA request)

        __HAL_TIM_DISABLE_DMA(_step_, _hardware_config_.EnbTIMDMA);

        //  2nd trigger a TIMER register update:
        _step_->Instance->EGR =
                ( TIM_EGR_UG | _hardware_config_.PulsEGRBit );
        // Trigger an update of the Top level counter (EGR_UR), as well as the Pulse Generator
        // "PulsEGRBit" - to get DMA 1st entry copied, as well as the Counter Interrupt

        __HAL_TIM_CLEAR_FLAG(_step_, _hardware_config_.PulsSRBit);
            // Clear the STEP Pulse generator bit

        // Note that both the Pulse and Count status bits are cleared. The update flag is not
        // cleared as depending upon the timing of the hardware, the update interrupt will be
        // triggered. So system designed such that this interrupt is expected - no action will
        // occur as, specified step count would not be achieved.
        // 3rd re-enable all TIMER registers and DMAs
        __HAL_TIM_ENABLE_DMA(_step_, _hardware_config_.EnbTIMDMA);  // Link DMA again

        __HAL_TIM_CLEAR_IT(_step_, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(_step_, TIM_IT_UPDATE);
        //__HAL_TIM_ENABLE_IT(_step_, _hardware_config_.CounIntBit);
            // Only enable the counter interrupt. The TIM overflow interrupt, will be enabled
            // by the counter interrupt; this saves on CPU resources

        _step_->Instance->CCER  |= _output_compare_channel_;
            // Enable STEP pulse Output Compare
    }
}

void StepperTIM::handleIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the Stepper Timer update event.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, the internals of this class will then manage the enabling/disabling of these
 * interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 *
 * This function is only expecting to be triggered for the Timer update event, for Stepper control.
 *
 * Interrupt will only do "basic" things normally; i.e. count pulses. Once the number of requested
 * steps has been achieved the main parts of this function are enabled:
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
    if (  (__HAL_TIM_GET_IT_SOURCE(_step_, TIM_IT_UPDATE) == SET)  &&
          (__HAL_TIM_GET_FLAG(_step_, TIM_FLAG_UPDATE) != 0)  ) {

        if (_shd_form_.StpCount != 0) {         // So long as the Shadow step count is not zero
            _shd_form_.StpCount--;              // subtract by 1
        }

        updateModelPosition(  1  );             // Update the modelled position

        // Check to see if the 'TIM_IT_UPDATE' interrupt has been enabled, and the interrupt has
        // occurred 'TIM_FLAG_UPDATE' (note different name, as different registers!) then...
        if (_shd_form_.StpCount == 0) {                     // If Shadow Form step count is zero
            //__HAL_TIM_DISABLE_IT(_step_, TIM_IT_UPDATE);    // disable this interrupt

            if (_shd_form_.cMode == Mode::kVelocity) {            // If mode is "Velocity"
                // Assumes that for the current setup, GPIOs demand haven't changed, therefore no
                // need to re-call 'setShadowGPIO'...yet.

                if (_form_queue_.state() != kGenBuffer_Empty) {
                // Check for any new movement requests
                    uint16_t prevpoint = _form_queue_.output_pointer;
                        // Capture current position (so as to rewind queue)
                    _form_queue_.outputRead( &_shd_form_ );   // Read new data

                    if (_shd_form_.cMode != Mode::kVelocity) {  // If anything but "Velocity" mode
                        disableMotor();
                        _form_queue_.output_pointer      = prevpoint;   // Rewind Form Queue
                        startInterrupt();     // Re-check for any other move requests
                    }
                    else {  // Otherwise mode is Velocity, so update the GPIOs as per new shadow
                            // setup, and load new frequency.
                        __HAL_TIM_SET_AUTORELOAD(_step_, _shd_form_.Freq);
                        setShadowGPIO();

                        // Force an update of the registers
                        _step_->Instance->EGR = TIM_EGR_UG;
                } }
            }
            else {
            // If mode is not velocity - then disable everything, and check for any new requests.
                disableMotor();

                startInterrupt();   // Check for any other move requests
            }
        }

        __HAL_TIM_CLEAR_IT(_step_, TIM_FLAG_UPDATE);    // Clear the interrupt bit for the Stepper
                                                        // controller interrupt
    }
}

StepperTIM::~StepperTIM()
{
    // TODO Auto-generated destructor stub
}

