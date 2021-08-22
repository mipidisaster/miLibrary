/**************************************************************************************************
 * @file        Stepper.h
 * @author      Thomas
 * @brief       Header file for the Stepper Driver Class handle
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
 *      Call Class Stepper to initialise the class, the arguments requires for construction are:
 *          Timer/DMA/Output Compare output for "Pulse"
 *          Output Compare channel to count number of "Pulses" (Interrupt)
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
 *          ".handleTIMxUpIRQ"      - Interrupt handler to be placed within the "TIMx_UP" request
 *                                    used to handle transitions between new requests + majority
 *                                    of control for stepper
 *          ".handleTimxCCIRQ"      - Interrupt handler to be placed within the "TIMx_CC" request
 *                                    used to handle counting of the pulses generated.
 *
 *      Following functions are protected so will only work for class internals. They contain
 *      basic handling of the hardware:
 *          ".setShadowGPIO"        - Setup GPIOs are per current request
 *          ".startInterrupt"       - Check for any new movement requests, whilst class is
 *                                    "Disabled"
 *
 *      Class is based/relient upon the use of interrupts - which are used to manage the change
 *      between new movement requests, as well as counting number of pulses.
 *
 *  [#] How Stepper PULSE is generated, and managed
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      So as to minimise the amount of CPU usage for management of interfacing with a Stepper
 *      driver, the STM32 embedded devices needs to be configured with the following:
 *          Linked TIMER needs to be dedicated solely to the Stepper Driver (other channels of
 *          timer cannot be used for anything else....yet)
 *          Output Compare - Driving the specific GPIO connected to driver "STEP" input
 *          Output Compare - None Output, this will be used within the class internals to be
 *                           triggered at the same time as the rising edge of the "PULSE"
 *
 *      The class system is based upon an interrupt being triggered at the base TIMER overflow,
 *      as well as for the interrupt triggered by the "Output Compare" to count pulses - 
 *      Referred to as "Counter Interrupt".
 *
 *      If the class is in "Position" mode, then the Overflow interrupt will do nothing (it is
 *      actually disabled) until the specific number of steps have been achieved (counted by 
 *      "Counter Interrupt"). Once specified step count has been achieved, the "Counter Interrupt"
 *      will enable the Overflow interrupt, which will then disable all outputs/interrupts and
 *      check for any new movement requests.
 *
 *      If the class is in "Velocity" mode, again the Overflow interrupt will be disabled; the
 *      system assumes that the GPIOs will not need to be driven continually. The "Counter 
 *      Interrupt" will still be counting the pulses (similar to "Position" mode), it will also
 *      be checking that there are no new movement requests. Once it detects any new requests, it
 *      will then enable the Overflow interrupt, which will setup the new request - note however
 *      when a new velocity is triggered the GPIO values for the request will not be set until the
 *      NEXT pulse (to ensure that the requested configuration aligns with the new pulse
 *      frequency; it will take 1 update of the TIMER before new frequency is used).
 *
 *      DIAGRAM OF OUTPUT PULSES:
 *                                      __                                  __
 * 11 |                              __|  |                              __|  |
 * 10 |                           __|     |                           __|     |
 *  9 |                        __|        |                        __|        |
 *  8 |                     __|           |                     __|           |
 *  7 |                  __|              |                  __|              |
 *  6 |               __|                 |               __|                 |               __|
 *  5 |            __|                    |            __|                    |            __|
 *  4 |         __|                       |         __|                       |         __|
 *  3 |      __|                          |      __|                          |      __|
 *  2 |   __|                             |   __|                             |   __|
 *  1 |__|________________________________|__|________________________________|__|______________
 *       |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
 *              __                                  __                                  __
 * PULSE  _____|  |________________________________|  |________________________________|  |_____
 *
 * OvFlo  ________________________________|___________________________________|_________________
 *
 * CInt   _____|___________________________________|___________________________________|________
 *
 * Count    0  |     1                             |     2                             |     3
 *        ________________________________       _____________________________       ___________
 * Auxi   ________________________________X-----X_____________________________X-----X___________
 *
 * Pulse configured such that first toggle occurs when TIMER = 50, and second toggle occurs when
 * it equals 55 (DMA manages this)
 * CInt     - Counter Interrupt, triggers at same time as rising edge of PULSE. Where the count is
 *            updated
 *            Additionally, the CPU will take some time to manage this interrupt, as well as
 *            compute the step count. Therefore the Fastest Frequency needs to allow sufficient
 *            time for this to occur.
 *
 * OvFlo    - Interrupt will only be called when the current position has been achieved or a new
 *            movement request has been made. Note that there is a delay between the interrupt 
 *            occuring and the hardware state updating. This delay needs to be accounted for in
 *            the time taken for the rising edge of PULSE.
 *
 * >> Additional note <<
 * Due to the hardware timer running faster then the CPU, there is a possibility that the Overflow
 * interrupt will be triggered twice, during the phase of disabling and then re-enabling
 * interrupts. This has been accounted for within the design of the Stepper class - hence the
 * "Counter Interrupt" and "OvFlo" being based upon this count.
 * So real world plot of the above, will include additional "OvFlo" pulses during phases of change
 * of mode - Position and Disabled. Velocity to Velocity, no double tap will appear.
 *
 * The fastest frequency is user imposed as 100 counts.
 * 
 *************************************************************************************************/
#ifndef STEPPER_H_
#define STEPPER_H_

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
#include FilInd_DMAPe__HD               // Allow use of the DMA class

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class Stepper : public DMAPeriph {
private:
     static const uint16_t  kdefault_profile_width      = 5;
     static const uint8_t   kprofile_array_pulse_depth  = 1;

     static const uint8_t   kform_size                  = 10;
     static const uint8_t   knumber_micro_pins          = 3;

     static const uint16_t  kmax_frequency              = 100;
     static const uint16_t  kcount_limit                = 0xFFFF;
     /*  Default width of the STEP pulse, units will be 'counts'
      *  Form size used for the internal buffer, for Stepper demands
      *  Number of pins used for defining the MicroSteps -> 1/2 step, 1/4 step, 1/8 step, etc.
      *  Maximum Frequency that this class can support, it needs to be sized so as to allow
      *  sufficient time to calculate the next pulse, and put into the DMA (so as to maintain
      *  desired pulse). Additionally, it also needs to allow sufficient to for any other stepper
      *  classes to calculate pulses at full speed.
      *  Based upon logic analyser, the interrupt time is ~5us (going to 10us for a state change),
      *  so with maximum pulse of 100 (assumed 'us') this will allow 20 Stepper classes to work
      *  concurrently.
      */

/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "Stepper::" followed by the type.
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
    private:
        // The following GPIO pins, are synchronous with the rising edge of the STEP pin.
        GPIO        *_nreset_pin_;  // Provide a pointer to the GPIO class, which is connected to
                                    // the RESET pin -  "LOW" = RESET
        GPIO        *_direction_;   // Provide a pointer to the GPIO class, which is connected to
                                    // the DIR pin
        GPIO        *_micro_step_;  // Provide a pointer to the GPIO class(es), which are
                                    // connected to the MS/MicroStep pins
        uint8_t     _micro_step_size_;          // Number of pins used for Microstep

    public:     // Following are shadow values which are the current active state of the Stepper
                // driver
        Form            _form_arry_[kform_size];    // Array to contain the Stepper request forms
        GenBuffer<Form> _form_queue_;               // GenBuffer for the Forms

        uint16_t    _shd_profile_[kprofile_array_pulse_depth * 2];
            // Profile to use for STEP pulse
        uint8_t     _pulse_run_;        // Number of pulses that will occur in a single DMA
                                        // transfer
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
    private:
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
        Stepper(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                int32_t FullRev, HrdSetup Config);
        /******************************************************************************************
         *   Pretty big constructor, as it contains:
         *      - Timer/DMA/channel to manage the STEP pulse (hardware)
         *         Note the "OCChannel" needs to be the Channel state shifted by the output
         *         channel
         *      - Interrupt channel, used for pulse management
         *         This channel doesn't need to contain the channel state (as not an output!)
         *      - The Reset and Direction pins
         *      - Step resolution pins (Microstep[], and size)
         *      - Define the maximum number of steps per Rev (to include smaller rotation per step
         *        (i.e. 1/16 step per STEP pulse)
         *      - Hardware configuration (DMA address link, etc).
         *****************************************************************************************/

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================

#else
//=================================================================================================

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "Stepper" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
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
    void startInterrupt(void);          // Enable the STEP pulse generator interrupt, if the
                                        // system is currently "DISABLED". Otherwise leave in
                                        // buffer
    void updatePulseDMA(uint16_t prevposition);          // Calculate the next pulse position, and implement in
                                        // array
    void updateModelPosition(void);     // Calculate the modelled position of the stepper motor
    void disableMotor(void);            // Function to be called when transitioning from one
                                        // position demand, to another
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

    void handleTIMxUpIRQ(void);         // Interrupt handler to be placed within the "TIMx_UP"
                                        // interrupt request, or interrupt triggered at base timer
                                        // overflow
    virtual ~Stepper();
};

#endif /* STEPPER_H_ */
