/**************************************************************************************************
 * @file        Stepper.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        09 Mar 2019
 * @brief       Source file for the Stepper Driver Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_Stppr__HD

void Stepper::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the Stepper class. To be called by all constructors.
 *************************************************************************************************/
    uint8_t i = 0;              // Parameter used to loop through arrays

    // Populate the Shadow Form with default parameters driving the Stepper Driver off
    this->ShdForm.Dir       = GPIO::State::LOW;     // Pull direction low
    this->ShdForm.nRst      = GPIO::State::LOW;     // Set RESET low = Reset
    for (i = 0; i != StpMaxMicroStepPins; i++) {    // Cycle through array of Microstep states
        this->ShdForm.Micr[i] = GPIO::State::LOW;   // and initialise GPIOs low
    }
    this->ShdForm.Freq      = 0;                    // Initialise the frequency to zero
    this->ShdForm.StpCount  = 0;                    // Initialise count to zero

    this->ShdForm.cMode     = Mode::Disabled;       // Indicate that stepper is disabled

    this->ShdCountConf.stpAmount    = 0;

    this->ShdCountConf.Pol          = CountPanel::Polarity::UP;

    this->ShdForm.CountConf = this->ShdCountConf;

    for (i = 0; i != StpBuffSize; i++) {            // Loop through entries within Form array
        this->FormArry[i]   = this->ShdForm;        // and set equal to shadow form (which is now
                                                    // in default state
    }

    this->ShdPfrl[0]    = StpDefStepPrfDly;                     // Put into profile delay for
                                                                // rising edge
    this->ShdPfrl[1]    = StpDefStepPrfDly + StpDefStepPrfWdt;  // Then calculate count for
                                                                // default width

    this->FormQueue.create(&this->FormArry[0], StpBuffSize);

    this->MicStpSiz     = 0;                // Initialise the number of Microstep pins to zero

    this->FullRev       = 0;                // Set the number of steps for full revolution to zero
    this->calcPos       = 0;                // Initialise pole Position calculation to zero
}

Stepper::Stepper() {
/**************************************************************************************************
 * Basic construction of Stepper class
 *************************************************************************************************/
    this->popGenParam();                    // Populate generic class parameters

}

void Stepper::create(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                     uint32_t CountChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                     int32_t FullRev, HrdSetup Config) {
/**************************************************************************************************
 * Creates a Stepper class specific for the STM32F device.
 *  Inputs include the TIMER handle which will be used primarily to control the stepper motor
 *   >>> Doesn't support another other outputs as of yet...  <<<
 *  Along with the Output Compare channel (which is the channel state shifted up by the channel)
 *        + channel for the output compare interrupt - used to count number of steps done
 *  The linked DMA is also required, to provide to the Output Compare output (alias "STEP") with
 *  the required sequence to generate the STEP pulse.
 *
 *  Then all the auxillary GPIO pins - Reset, and Microstep(s) < last one needs to also include
 *                                                               the number of pins for this
 *  The number of steps per revolution of the Stepper need to be provided in the format of
 *  hardware number of steps multiplied by the smallest step size:
 *      i.e. motor has 200 poles, but driver can provide a step of 1/16
 *           "FullRev"  = 200 x 16 = 3,200 steps
 *
 *  Lastly the Hardware Setup structure is required. This should be configured for the specific
 *  embedded device (limited to STM32 devices currently). Including the bit positions and address
 *  to manage interrupts and the DMA.
 *************************************************************************************************/
    this->popGenParam();                    // Populate generic class parameters

    this->nResePin  = nReset;               // Link the Reset pin to the class internals
    this->Dirction  = DIR;                  // Link the Direction pin to the class internals

    this->MicrStep  = MicStp;               // Link input Microstep array to class internals
    this->MicStpSiz = McrStp;               // Put number of microstep pins into array

    this->FullRev   = FullRev;              // Copy Full Revolution number to class internals

    this->STEP      = STEP_TIM;             // Link the STEP timer to class internals
    this->DMA_stp   = STEP_DMA;             // Link timer DMA to class internals
    this->OtpCopChnl= OCChannel;            // Provide channel output for "Output Compare"
                                            // (OUTPUT)
    this->HrdwreCfg = Config;               // Link Hardware setup to class internals

    this->calcPos   = 0;                    // Initialise the calculated position to 0

    // Configure Timer hardware for use with this Stepper class:
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Configure Timer and Linked DMA
    __HAL_DMA_DISABLE(this->DMA_stp);       // Ensure that DMA is disabled for initial setup
    HAL_DMA_Start(this->DMA_stp,    (uint32_t)&this->ShdPfrl[0],
                                    this->HrdwreCfg.PulseDMAAddress,
                                    2);
    // Link DMA to the internal Shadow Profile, use Hardware setup DMA Address to link to correct
    // Timer hardware register (Memory to Peripheral setup is required)
    //                          ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DMA also needs to be configured as a circular buffer
    //                                      ~~~~~~~~~~~~~~~

    __HAL_TIM_ENABLE_DMA(this->STEP, this->HrdwreCfg.EnbTIMDMA);    // Enable the link between
                                                                    // Timer and DMA
    __HAL_DMA_ENABLE(this->DMA_stp);                                // Then enable the DMA


    __HAL_TIM_SET_COMPARE(this->STEP, CountChannel, StpDefStepPrfDly);
    // Setup the Timer counter "Output Compare" threshold to same threshold as initial STEP pulse

    __HAL_TIM_SET_AUTORELOAD(this->STEP, 0);        // Ensure that the frequency is set to zero

    // Now trigger an update of the TIMER hardware shadow registers (by driving the EGR bits True)
    //  This needs to be done with the DMA enabled, such that when the first threshold is achieved
    //  DMA will then update the Output Compare threshold with the next - completing the PULSE

    this->STEP->Instance->EGR =
            ( TIM_EGR_UG | this->HrdwreCfg.PulsEGRBit | this->HrdwreCfg.CounEGRBit );
    // Trigger an update of the Top level counter (EGR_UR), as well as the Pulse Generator
    // "PulsEGRBit" - to get DMA 1st entry copied, as well as the Counter Interrupt
    // "CounEGRBit" - to copy across update bit

    __HAL_TIM_CLEAR_FLAG(this->STEP, TIM_FLAG_UPDATE);              // Clear the update bit
    __HAL_TIM_CLEAR_FLAG(this->STEP, this->HrdwreCfg.PulsSRBit);    // Clear the STEP pulse
                                                                    // generator bit
    __HAL_TIM_CLEAR_FLAG(this->STEP, this->HrdwreCfg.CounSRBit);    // Clear the STEP count set
                                                                    // bit

    // Bits cleared such that interrupts are not triggered.
    // Now enable the TIMER - the outputs and interrupts have not been enabled yet. This is
    // intentional, as do not want to have the outputs being driven during initialisation.
    if (IS_TIM_BREAK_INSTANCE(this->STEP->Instance) != RESET) {
        __HAL_TIM_MOE_ENABLE(this->STEP);
    }

    __HAL_TIM_ENABLE(this->STEP);
}

Stepper::Stepper(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                 uint32_t CountChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                 int32_t FullRev, HrdSetup Config) {
/**************************************************************************************************
 * Creates a Stepper class specific for the STM32F device.
 *  Inputs include the TIMER handle which will be used primarily to control the stepper motor
 *   >>> Doesn't support another other outputs as of yet...  <<<
 *  Along with the Output Compare channel (which is the channel state shifted up by the channel)
 *        + channel for the output compare interrupt - used to count number of steps done
 *  The linked DMA is also required, to provide to the Output Compare output (alias "STEP") with
 *  the required sequence to generate the STEP pulse.
 *
 *  Then all the auxillary GPIO pins - Reset, and Microstep(s) < last one needs to also include
 *                                                               the number of pins for this
 *  The number of steps per revolution of the Stepper need to be provided in the format of
 *  hardware number of steps multiplied by the smallest step size:
 *      i.e. motor has 200 poles, but driver can provide a step of 1/16
 *           "FullRev"  = 200 x 16 = 3,200 steps
 *
 *  Lastly the Hardware Setup structure is required. This should be configured for the specific
 *  embedded device (limited to STM32 devices currently). Including the bit positions and address
 *  to manage interrupts and the DMA.
 *************************************************************************************************/
    this->create(STEP_TIM, STEP_DMA, OCChannel, CountChannel,
                 nReset, DIR, MicStp, McrStp, FullRev, Config);
}

void Stepper::SetShadowGPIO(void) {
/**************************************************************************************************
 * Configure the internally linked GPIO pins (RESET, DIR, MicroStep) as per the Shadow Form
 *************************************************************************************************/
    this->nResePin->setValue(this->ShdForm.nRst);   // Set the RESET pin to desired state
    this->Dirction->setValue(this->ShdForm.Dir);    // Set the DIR pin to desired state

    uint8_t i = 0;          // Create variable to loop through MicroStep GPIO pins
    for (i = 0; i != this->MicStpSiz; i++) {                // Loop through Micro pins
        this->MicrStep[i].setValue(this->ShdForm.Micr[i]);  // and set to desired value
    }

    this->ShdCountConf = this->ShdForm.CountConf;   // Bring count configuration to shadow
                                                    // controller
    // Set at same time as GPIO, to ensure count is synched correctly.
}

void Stepper::InterruptSetup(void) {
/**************************************************************************************************
 * Function checks state of the class mode, and as to whether there is any new movement requests
 * within the buffer.
 * if the mode is "Disabled", and a new request exists - then function will kick off the desired
 * stepper movement.
 *
 * If this condition is not set, then function exits.
 *************************************************************************************************/
    if ( (this->ShdForm.cMode == Mode::Disabled) &&
         (this->FormQueue.State() != GenBuffer_Empty) ) {
        this->FormQueue.OutputRead( &this->ShdForm );           // Retrieve next request and put
                                                                // into shadow form
        this->SetShadowGPIO();                                  // Setup GPIOs are per active form

        __HAL_TIM_SET_AUTORELOAD(this->STEP, this->ShdForm.Freq);   // Update with the requested
                                                                    // frequency
        // Now go about updating the timer shadow registers, assumes that the interrupts and
        // outputs are already switched off, so now:
        //  1st Disable link to DMA (do not want timer update to trigger a DMA request)

        __HAL_TIM_DISABLE_DMA(this->STEP, this->HrdwreCfg.EnbTIMDMA);

        //  2nd trigger a TIMER register update:
        this->STEP->Instance->EGR =
                ( TIM_EGR_UG | this->HrdwreCfg.PulsEGRBit | this->HrdwreCfg.CounEGRBit );
        // Trigger an update of the Top level counter (EGR_UR), as well as the Pulse Generator
        // "PulsEGRBit" - to get DMA 1st entry copied, as well as the Counter Interrupt
        // "CounEGRBit" - to copy across update bit

        __HAL_TIM_CLEAR_FLAG(this->STEP, this->HrdwreCfg.PulsSRBit);    // Clear the STEP pulse
                                                                        // generator bit
        __HAL_TIM_CLEAR_FLAG(this->STEP, this->HrdwreCfg.CounSRBit);    // Clear the STEP count set
                                                                        // bit

        // Note that both the Pulse and Count status bits are cleared. The update flag is not
        // cleared as depending upon the timing of the hardware, the update interrupt will be
        // triggered. So system designed such that this interrupt is expected - no action will
        // occur as, specified step count would not be achieved.
        // 3rd re-enable all TIMER registers and DMAs
        __HAL_TIM_ENABLE_DMA(this->STEP, this->HrdwreCfg.EnbTIMDMA);    // Link DMA again

        __HAL_TIM_ENABLE_IT(this->STEP, TIM_IT_UPDATE);                 // Enable Update interrupt
        __HAL_TIM_ENABLE_IT(this->STEP, this->HrdwreCfg.CounIntBit);    // Enable Step counter
                                                                        // interrupt
        this->STEP->Instance->CCER  |= this->OtpCopChnl;
            // Enable STEP pulse Output Compare
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
 *  Pol     - Defines the polarity (Count UP, or DOWN)
 *  StppPul - Defines the count per STEP
 *************************************************************************************************/
    Form tmpForm;                                   // Temporary form prior to entry into buffer
    uint8_t i = 0;                                  // Variable to loop through Microstep pins

    tmpForm.Dir     = DIR;                          // Copy across the desired Direction state
    tmpForm.nRst    = GPIO::State::HIGH;            // Set RESET to "HIGH" - No RESET

    for (i = 0; i != StpMaxMicroStepPins; i++) {    // Loop through the Microstep pins
                                                    // (pre-processor size)
        if (i >= this->MicStpSiz) {                 // If loop is greater than defined number of
                                                    // GPIO pins
            tmpForm.Micr[i]     = GPIO::State::LOW; // Default to "LOW"
        }
        else {                                      // If within defined number of GPIOs
            if (MicroStp & 1)                       // If 1st bit is set to "1"
                tmpForm.Micr[i] = GPIO::State::HIGH;// Set state to "HIGH"
            else                                    // OTHERWISE
                tmpForm.Micr[i] = GPIO::State::LOW; // Set state to "LOW"

            MicroStp >>= 1;     // Shift down by 1 bit
    } }

    tmpForm.CountConf.Pol           = Pol;          // Copy across both Polarity and Count per
    tmpForm.CountConf.stpAmount     = StpprPul;     // step into Form Counter Configuration

    tmpForm.Freq        = Freq;                     // Copy across Frequency into form
    tmpForm.StpCount    = PulseCount;               // Copy across number of pulses into form
    tmpForm.cMode       = Mode::Position;           // Set mode to "Position"

    this->FormQueue.InputWrite( tmpForm );          // Put temporary form into queue
    this->InterruptSetup();                         // Call interrupt setup function
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
    Form tmpForm;                                   // Temporary form prior to entry into buffer
    uint8_t i = 0;                                  // Variable to loop through Microstep pins

    tmpForm.Dir     = DIR;                          // Copy across the desired Direction state
    tmpForm.nRst    = GPIO::State::HIGH;            // Set RESET to "HIGH" - No RESET

    for (i = 0; i != StpMaxMicroStepPins; i++) {    // Loop through the Microstep pins
                                                    // (pre-processor size)
        if (i >= this->MicStpSiz) {                 // If loop is greater than defined number of
                                                    // GPIO pins
            tmpForm.Micr[i]     = GPIO::State::LOW; // Default to "LOW"
        }
        else {                                      // If within defined number of GPIOs
            if (MicroStp & 1)                       // If 1st bit is set to "1"
                tmpForm.Micr[i] = GPIO::State::HIGH;// Set state to "HIGH"
            else                                    // OTHERWISE
                tmpForm.Micr[i] = GPIO::State::LOW; // Set state to "LOW"

            MicroStp >>= 1;     // Shift down by 1 bit
    } }

    tmpForm.CountConf.Pol           = Pol;          // Copy across both Polarity and Count per
    tmpForm.CountConf.stpAmount     = StpprPul;     // step into Form Counter Configuration

    tmpForm.Freq        = Freq;                     // Copy across Frequency into form
    tmpForm.StpCount    = 0;                        // As in "Velocity" mode no step count is
                                                    // required
    tmpForm.cMode       = Mode::Velocity;           // Set mode to "Velocity"

    this->FormQueue.InputWrite( tmpForm );          // Put temporary form into queue
    this->InterruptSetup();                         // Call interrupt setup function
}

void Stepper::IRQUPHandler(void) {
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
 *      If current Mode is "Position", then interrupt disables all interrupts, and outputs,
 *                                     mode is then put into "Disabled".
 *                                     Function "InterruptSetup" is then called to check for any
 *                                     other movement requests.
 *
 *      If current Mode is "Velocity", then function will setup the GPIO are per current shadow
 *                                     form ("SetShadowGPIO").
 *                                     If there is no new movement requests then interrupt exits
 *                                     If there is a new movement request so long as its
 *                                     "Velocity", then new "Velocity" is selected
 *                                     If new movement is not "Velocity", then interrupt disables
 *                                     all interrupts, and outputs, mode is then put into
 *                                     "Disabled".
 *                                     Function "InterruptSetup" is then called to check for any
 *                                     other movement requests.
 *
 *      No other interrupts are currently supported. However function will not run if the
 *      interrupt was not caused by a 'TIM_IT_UPDATE' event
 *************************************************************************************************/
    if (  (__HAL_TIM_GET_IT_SOURCE(this->STEP, TIM_IT_UPDATE) == SET)  &&
          (__HAL_TIM_GET_FLAG(this->STEP, TIM_FLAG_UPDATE) != 0)  ) {
        // Check to see if the 'TIM_IT_UPDATE' interrupt has been enabled, and the interrupt has
        // occurred 'TIM_FLAG_UPDATE' (note different name, as different registers!) then...
        __HAL_TIM_CLEAR_IT(this->STEP, TIM_FLAG_UPDATE);// Clear the interrupt bit for the Stepper
                                                        // controller interrupt
        if (this->ShdForm.StpCount == 0) {                  // If Shadow Form step count is zero
            if (this->ShdForm.cMode  == Mode::Position) {   // and current mode is "Position"
                this->ShdForm.cMode     = Stepper::Disabled;        // Set mode to "Disabled"
                __HAL_TIM_DISABLE_IT(this->STEP, TIM_IT_UPDATE);    // Disable the stepper
                                                                    // controller interrupt
                __HAL_TIM_DISABLE_IT(this->STEP, this->HrdwreCfg.CounIntBit);   // Disable Step
                                                                                // Count Interrupt
                this->STEP->Instance->CCER &= ~(this->OtpCopChnl);
                        // Clear the output enabling pin for the STEP pulse

                this->InterruptSetup(); // Check for any other move requests
            }
            else if (this->ShdForm.cMode == Mode::Velocity) {       // If mode is "Velocity"
                this->SetShadowGPIO();                              // Set GPIO as per Shadow

                if (this->FormQueue.State() != GenBuffer_Empty) {   // Check for any new movement
                                                                    // requests
                    uint32_t prevpoint = this->FormQueue.output_pointer;
                        // Capture current position (so as to rewind queue)
                    this->FormQueue.OutputRead( &this->ShdForm );   // Read new data

                    if (this->ShdForm.cMode != Mode::Velocity) {    // If not a velocity mode
                        this->ShdForm.cMode     = Stepper::Disabled;        // Then set mode to
                                                                            // "Disabled"
                        __HAL_TIM_DISABLE_IT(this->STEP, TIM_IT_UPDATE);
                                // Disable the stepper controller interrupt
                        __HAL_TIM_DISABLE_IT(this->STEP, this->HrdwreCfg.CounIntBit);
                                // Disable Step Count Interrupt
                        this->STEP->Instance->CCER &= ~(this->OtpCopChnl);
                                // Clear the output enabling pin for the STEP pulse

                        this->FormQueue.output_pointer      = prevpoint;    // Rewind Form Queue
                        this->InterruptSetup();     // Check for any other move requests
                    }
                    else {  // Otherwise Mode is Velocity, so load up frequency, and then
                            // next loop will drive the GPIO aux. signals
                        __HAL_TIM_SET_AUTORELOAD(this->STEP, this->ShdForm.Freq);
                            // Update with the requested frequency
                } }
            }
            else {
                this->ShdForm.cMode     = Stepper::Disabled;        // Set mode to "Disabled"
                __HAL_TIM_DISABLE_IT(this->STEP, TIM_IT_UPDATE);    // Disable the stepper
                                                                    // controller interrupt
                __HAL_TIM_DISABLE_IT(this->STEP, this->HrdwreCfg.CounIntBit);
                        // Disable Step Count Interrupt
                this->STEP->Instance->CCER &= ~(this->OtpCopChnl);
                        // Clear the output enabling pin for the STEP pulse
            }
        }
    }
}

void Stepper::IRQCounterCCHandler(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the Stepper Timer Output Compare event
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, the internals of this class will then manage the enabling/disabling of these
 * interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 *
 * This function is only expecting to be triggered for the Timer Output Compare event linked to
 * the STEP counter
 *
 * First part of function will subtract the Shadow step count so long as it is not equal to zero.
 * Second part, will subtract/add the specific step amount per pulse as per the current "Count
 * Configuration". This is then limited to be within the "FullRev" defined at class construction.
 *
 *      No other interrupts are currently supported. However function will not run if the
 *      interrupt was not caused by a '.CounIntBit' event
 *************************************************************************************************/
    if (  (__HAL_TIM_GET_IT_SOURCE(this->STEP, this->HrdwreCfg.CounIntBit) == SET)  &&
          (__HAL_TIM_GET_FLAG(this->STEP, this->HrdwreCfg.CounSRBit) != 0)  ) {
        // Check to see if the '.CounIntBit' interrupt has been enabled, and the interrupt has
        // occurred '.CounSRBit' (note different name, as different registers!) then...
        __HAL_TIM_CLEAR_IT(this->STEP, this->HrdwreCfg.CounSRBit);
            // Clear the interrupt bit for the Stepper count interrupt
        if (this->ShdForm.StpCount != 0) {          // So long as the Shadow step count is not zero
            this->ShdForm.StpCount--;               // subtract by 1
        }

        if (this->ShdCountConf.Pol == CountPanel::Polarity::UP) {   // If current Count
                                                                    // Configuration is count "UP"
            this->calcPos = (this->calcPos + this->ShdCountConf.stpAmount) % this->FullRev;
                // Then add the specified count per STEP to the "calcPos". This is then limited to
                // the maximum number of steps per revolution
        }
        else {  // OTHERWISE polarity is "DOWN"
            this->calcPos = ( (this->calcPos - this->ShdCountConf.stpAmount) + this->FullRev )
                               % this->FullRev;
                // Then subtract the specified count per STEP to the "calcPos". This is then
                // limited to the maximum number of steps per revolution
        }
    }
}

Stepper::~Stepper()
{
    // TODO Auto-generated destructor stub
}

