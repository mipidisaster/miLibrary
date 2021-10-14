/**************************************************************************************************
 * @file        StepperCore.h
 * @author      Thomas
 * @brief       Header file for the Stepper Driver Class handle (Core control)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class supports only the STM32 embedded devices, the initialisation of which is common across
 * the series of devices.
 *  >> Note support for Raspberry Pi is unlikely however, might happen WATCH THIS SPACE!! <<
 *
 *      This is the core functionality of the Stepper class, and is intended NOT to be called
 *      outright. It is expected to be called via the top level versions - "StepperDMA", or
 *      "StepperTIM". The difference between these is the way that the Stepper Pulses are managed;
 *      "StepperDMA" makes use of the DMA Transmit Complete interrupt, whereas "StepperTIM" makes
 *      use of the Timers Overflow interrupt - look into each of these to see which implementation
 *      is best for your application.
 *
 *      This CORE class requires the following arguments for construction (which is the same as
 *      the top levels):
 *          Timer/DMA/Output Compare output for "Pulse"
 *          Reset/Direction/Microstep pins (based upon the GPIO class)
 *          Number of poles per revolution of the stepper motor (this includes the smallest
 *          rotation per step, i.e. 1/16 step per PULSE)
 *          Hardware Configuration (DMA Address link, etc.)
 *
 *      Once construction is done, the parameters which are requires for this class to function
 *      are:
 *          ".forceSTOP"            - Forces a STOP of the motor
 *          ".newPosition"          - Request a new position (number of steps, at a specific
 *                                    frequency)
 *          ".newVelocity"          - Request a new velocity (just a frequency of steps)
 *         V".handleIRQ"            - This is a blank virtual function, and will be populated by
 *                                    the top level versions.
 *
 *      Following functions are protected so will only work for class internals. They contain
 *      basic handling of the hardware:
 *          ".setShadowGPIO"        - Setup GPIOs are per current request
 *         V".startInterrupt"       - This is a blank virtual function, and will be populated by
 *                                    the top level versions.
 *          ".updateModelPosition"  - This function is called to calculate the expected position
 *                                    of the stepper motor, based upon how many pulses have been
 *                                    triggered, and how much of a "step" each is.
 *                                    Note - Maximum number of pulses per rotation will be based
 *                                    on the smallest step possible.
 *      i.e. motor has 200 poles, but driver can provide a step of 1/16
 *              "full_revolution"  = 200 x 16 = 3,200 steps
 *
 *         V".disableMotor"         - This is a blank virtual function, and will be populated by
 *                                    the top level versions.
 *
 *      Class is based/relient upon the use of interrupts - which are used to manage the change
 *      between new movement requests, as well as counting number of pulses.
 *
 *  [#] How Stepper PULSE is generated, and managed
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      So as to minimise the amount of CPU usage for management of interfacing with a Stepper IC,
 *      The STM32 embedded devices needs to be configured with the following:
 *          Linked TIMER, with the Output Compare option enabled (in Toggle mode)
 *          DMA to be linked to the channel of Output Compare, so as to populate with the new time
 *          to toggle the output.
 *
 *      The CORE functionality will make use of an interrupt to count how many pulses have been
 *      triggered, as well as to determine if there are any new movement requests (as well as any
 *      other tasks which are needed to manage the hardware correctly, the specific details of
 *      which will be within the specific top level class).
 *
 *      The basics of the interrupt as such that:
 *          If in the "Position" mode, then the interrupt will count the number of pulses, and
 *          once this number reaches the desired number of steps, it will then disable all
 *          outputs/interrupts and check for any new movement requests.
 *
 *          If in the "Velocity" mode, then it will once again count the number of pulses (so as to
 *          determine the expected position), however, once it detects a new movement request, it
 *          will apply this new request straight away.
 * 
 *************************************************************************************************/
#ifndef STEPPERCORE_H_
#define STEPPERCORE_H_

#include "FileIndex.h"
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

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class StepperCore {
protected:
     static const uint16_t  kdefault_profile_width      = 5;

     static const uint8_t   kform_size                  = 10;
     static const uint8_t   knumber_micro_pins          = 3;

     static const uint16_t  kmax_frequency              = 100;
     /*  Default width of the STEP pulse, units will be 'counts'
      *
      *  Form size used for the internal buffer, for Stepper demands
      *  Number of pins used for defining the MicroSteps -> 1/2 step, 1/4 step, 1/8 step, etc.
      *
      *  Maximum Frequency that this class can support, it needs to be sized so as to allow
      *  sufficient time to calculate the next pulse, and put into the DMA (so as to maintain
      *  desired pulse). Additionally, it also needs to allow sufficient to for any other stepper
      *  classes to calculate pulses at full speed.
      */
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "StepperCore::" followed by the type.
 *************************************************************************************************/
public:
        enum Mode     : uint8_t { kDisabled = 0, kPosition = 1, kVelocity = 2 };
        // Mode for the Stepper logic:
        /******************************************************************************************
         * kDisabled -  Timer output and interrupts are disabled. This is the initial state of
         *              class at initialisation.
         *
         * kPosition -  Where the number of steps to be made (and frequency) is defined and copied
         *              into the shadow variables. Once the number of steps has been achieved,
         *              device will stop stepping.
         *              Then will check buffer entries for new data (based on "DataStatus")
         *
         * kVelocity -  Where the number of steps is NOT defined, however frequency is. Then the
         *              stepper will simply keep stepping at the defined rate.
         *              Any changes to velocity, will be synced to align with any changes to
         *              Reset/Microstep pin changes - which will occur on the interrupt after the
         *              interrupt that detects the buffer updates (sync will be done via
         *              shdStpCount)
         *****************************************************************************************/

        struct CountPanel {         // Structure used to control the class internal calculation
                                    // for determining current pole position.
            uint32_t    stpAmount;                              // Amount to count up/down for
                                                                // STEP
            enum Polarity : uint8_t { kUp = 0, kDown = 1} Pol;  // Direction for counting, either
                                                                // Up   - Add
                                                                // Down - Subtract
        };

        struct HrdSetup {                   // Hardware structure to allow class to interface to
                                            // multiple embedded devices.
            // Note, naming is designed to help determine which bit position to use:
            // i.e. "PulsEGRBit" use the bit position for the EGR register, etc.
            uint32_t    PulseDMAAddress;    // Address to link the DMA to (Memory to Peripheral)
            uint32_t    EnbTIMDMA;          // Bit position to enable the link between Timer and
                                            // DMA
            uint32_t    PulsEGRBit;         // Bit position to trigger a Event update for the Pulse
                                            // output (what every its channel is)
            uint32_t    PulsSRBit;          // Bit position to clear the set event bit for the
                                            // Pulse
        };

        typedef struct {                    // Stepper controller Form structure, used to manage
                                            // changing position/velocity with interrupt and DMA
            GPIO::State Dir;                        // Define the GPIO state for DIRECTION pin
            GPIO::State nRst;                       // Define the GPIO state for Reset pin
            GPIO::State Micr[knumber_micro_pins];   // Define the GPIO state(s) for Microstep pins
            uint16_t    Freq;                       // STEP frequency
            volatile uint32_t    StpCount;          // Number of steps to do (only used in
                                                    // 'Position' mode
            Mode        cMode;                      // Mode to go into
            CountPanel  CountConf;                  // Counter configuration for desired movement
        }   Form;

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        // The following GPIO pins, are synchronous with the rising edge of the STEP pin.
        GPIO        *_nreset_pin_;  // Provide a pointer to the GPIO class, which is connected to
                                    // the RESET pin -  "LOW" = RESET
        GPIO        *_direction_;   // Provide a pointer to the GPIO class, which is connected to
                                    // the DIR pin
        GPIO        *_micro_step_;  // Provide a pointer to the GPIO class(es), which are
                                    // connected to the MS/MicroStep pins
        uint8_t     _micro_step_size_;          // Number of pins used for Microstep

                // Following are shadow values which are the current active state of the Stepper
                // driver
        Form            _form_arry_[kform_size];    // Array to contain the Stepper request forms
        GenBuffer<Form> _form_queue_;               // GenBuffer for the Forms

        CountPanel  _shd_count_conf_;   // Shadow (active) Count configuration
        Form        _shd_form_;         // Shadow entry of the controlling signals which need to be
                                        // synced with the rising edge of STEP
    public:
        uint32_t   full_revolution; // Defines the maximum number of steps per revolution of
                                    // stepper (needs to include the driver smallest step rate,
                                    // i.e. Motor has 200 poles, driver can allow 1/16 step per
                                    // pulse, therefore this would be 200 x 16 = 3,200
        volatile int32_t    calc_position;  // Current calculated position of the stepper pole.
                                            // Units 'steps'

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
    protected:
        void popGenParam(void);         // Populate generic parameters for the class

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    protected:
        TIM_HandleTypeDef   *_step_;    // Hardware controlled TIMER configured in "Output
                                        // Compare" mode, toggling the STEP GPIO pin
        DMA_HandleTypeDef   *_dma_stp_; // DMA linked to the specific TIM + channel for the
                                        // "Output Compare" - Output
        uint32_t            _output_compare_channel_;
                                        // Channel for the TIM "Output Compare" signal (OUTPUT)
                                        // This contains the Channel state shifted up by the
                                        // Output channel

        HrdSetup    _hardware_config_;  // Defined at class construction time

    public:
        StepperCore(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                    GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                    int32_t FullRev, HrdSetup Config);
        /******************************************************************************************
         *   Pretty big constructor, as it contains:
         *      - Timer/DMA/channel to manage the STEP pulse (hardware)
         *         Note the "OCChannel" needs to be the Channel state shifted by the output
         *         channel
         *      - The Reset and Direction pins
         *      - Step resolution pins (Microstep[], and size)
         *      - Define the maximum number of steps per Rev (to include smaller rotation per step
         *        (i.e. 1/16 step per STEP pulse)
         *      - Hardware configuration (DMA address link, etc).
         *****************************************************************************************/

        virtual ~StepperCore();

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================

#else
//=================================================================================================

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "StepperCore" class, which are generic; this
 *  means are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>      DIRECT HARDWARE SETUP FUNCTIONS      <<<
             *   -----------
             * Following functions are to be hidden as link directly to the hardware. Which the
             * class is designed to "hide"
             *************************************************************************************/
    void setShadowGPIO(void);           // Set the GPIO pins as per shadow.
    virtual void startInterrupt(void) {};   // Enable the STEP pulse generator interrupt, if the
                                            // system is currently "DISABLED". Otherwise leave in
                                            // buffer
                                            // (BLANK IN THIS CLASS)

    void updateModelPosition(uint8_t number_pulses);    // Calculate the modelled position of the
                                                        // stepper motor

    virtual void disableMotor(void) {}; // Function to be called when transitioning from one
                                        // position demand, to another
                                        // (BLANK IN THIS CLASS)
public:     /**************************************************************************************
             * ==  PUBLIC   == >>>        CLASS INTERFACE FUNCTIONS        <<<
             *   -----------
             *  Visible functions used to request any new position or velocity of the stepper
             *  motor.
             *  Functions are designed in interrupt mode exclusively.
             *************************************************************************************/
    void forceSTOP(void);           // Build and force a request to stop motor movement

    void newPosition(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq, uint32_t PulseCount,
                     CountPanel::Polarity Pol, uint32_t StpprPul);
        // Build a new request to move stepper to a new position, and a specific velocity.
        // Puts class into "Position" mode

    void newVelocity(GPIO::State DIR, uint8_t MicroStp, uint16_t Freq,
                     CountPanel::Polarity Pol, uint32_t StpprPul);
        // Build a new request to move stepper at a specific velocity (indefinitely).
        // Puts class into "Velocity" mode

    virtual void handleIRQ(void) {};    // Interrupt Handle for Stepper Device
                                        // (BLANK IN THIS CLASS)
};

#endif /* STEPPERCORE_H_ */
