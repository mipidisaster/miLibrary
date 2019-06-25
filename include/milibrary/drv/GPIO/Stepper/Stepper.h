/**************************************************************************************************
 * @file        Stepper.h
 * @author      Thomas
 * @version     V2.1
 * @date        16 Jun 2019
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
 *          ".IRQUpHandler"         - Interrupt handler to be placed within the "TIMx_UP" request
 *                                    used to handle transitions between new requests + majority
 *                                    of control for stepper
 *          ".IRQCounterCCHandler"  - Interrupt handler to be placed within the "TIMx_CC" request
 *                                    used to handle counting of the pulses generated.
 *
 *      Following functions are protected so will only work for class internals. They contain
 *      basic handling of the hardware:
 *          ".SetShadowGPIO"        - Setup GPIOs are per current request
 *          ".InterruptSetup"       - Check for any new movement requests, whilst class is
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
 *      If the class is in "Position" mode, then the Overflow interrupt will do nothing until the
 *      specific number of steps have been achieved (counted by "Counter Interrupt"). Once
 *      specified step count has been achieved, interrupt will then disable outputs/interrupts and
 *      check for any new movement requests
 *
 *      If the class is in "Velocity" mode, then the Overflow interrupt will continually be
 *      driving the GPIO values to the requested amount - note however when a new velocity is
 *      triggered the GPIO values for that request will not be set until the NEXT pulse (to ensure
 *      that the requested configuration aligns with the new pulse frequency; it will take 1
 *      update of the TIMER before new frequency is used)
 *      Interrupt will also be checking for any new movement requests. If there is a new
 *      "Velocity" it will be pulled across directly (albeit at the new pulse, see paragraph
 *      above). If it is not "Velocity", the interrupt will disable outputs/interrupts and then
 *      reload new movement request (using same function as "Position")
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
 * Pulse configured such that first toggle occurs when TIMER = 3, and second toggle occurs when
 * it equals 4 (DMA manages this)
 * CInt     - Counter Interrupt, triggers at same time as rising edge of PULSE. Where the count is
 *            updated
 *            Additionally, the CPU will take some time to manage this interrupt, as well as
 *            compute the step count. Therefore the Fastest Frequency needs to allow sufficient
 *            time for this to occur.
 *
 * OvFlo    - Interrupt manages the Aux GPIOs, note that there is a delay between the interrupt
 *            occuring and the hardware state updating. This delay needs to be accounted for in
 *            the time taken for the rising edge of PULSE.
 *
 * >> Additional note <<
 * Due to the hardware timer running fastes then the CPU, there is a possibility that the Overflow
 * interrupt will be triggered twice, during the phase of disabling and then re-enabling
 * interrupts. This has been accounted for within the design of the Stepper class - hence the
 * "Counter Interrupt" and "OvFlo" being based upon this count.
 * So real world plot of the above, will include additional "OvFlo" pulses during phases of change
 * of mode - Position and Disabled. Velocity to Velocity, no double tap will appear.
 *
 * Based upon the points above, the PULSE profile has been defined as 30 and 2; 30 counts for 1st
 * toggle, and 32 for 2nd toggle. This should contain sufficient margain for any target device.
 * The fastest frequency is user imposed as 100 counts.
 * 
 *************************************************************************************************/
#ifndef STEPPER_H_
#define STEPPER_H_

#include "FileIndex.h"

#include FilInd_GENBUF_HD               // Allow use of GenBuffer template class
#include FilInd_GPIO___HD               // Allow use of GPIO class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// None

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
#define  StpBuffSize            03      // Maximum number of entries to put in buffer (made 3
                                        // so can determine when it is full or not, 2 would always
                                        // be full!)
#define  StpMaxMicroStepPins    03      // Define the maximum number of pins linked to the
                                        // "MicrStep", so can fully define internal array
#define  StpDefStepPrfDly       50      // Default delay from interrupt for rising edge of STEP
#define  StpDefStepPrfWdt       05      // Default width for STEP pulse

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class Stepper {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "Stepper::" followed by the type.
 *************************************************************************************************/
public:
        enum Mode     : uint8_t { Disabled = 0, Position = 1, Velocity = 2 };
        // Mode for the Stepper logic:
        /******************************************************************************************
         * Disabled -   Timer output and interrupts are disabled. This is the initial state of
         *              class at initialisation.
         *
         * Position -
         *              Where the number of steps to be made (and frequency) is defined and copied
         *              into the shadow variables. Once the number of steps has been achieved,
         *              device will stop stepping.
         *              Then will check buffer entries for new data (based on "DataStatus")
         *
         *  Velocity -
         *              Where the number of steps is NOT defined, however frequency is. Then the
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
            enum Polarity : uint8_t { UP = 0, DOWN = 1} Pol;    // Direction for counting, either
                                                                // UP   - Add
                                                                // DOWN - Subtract
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

            uint32_t    CounIntBit;         // Bit position to enable/disable the interrupt for
                                            // counting the STEP done (Output compare threshold is
                                            // the same as 'StpDefStepPrfDly'
            uint32_t    CounEGRBit;         // Bit position to trigger a Event update for the STEP
                                            // counter (what channel is used for this)
            uint32_t    CounSRBit;          // Bit position to clear the set event bit for the
                                            // Counter
        };

        typedef struct {                    // Stepper controller Form structure, used to manage
                                            // changing position/velocity with interrupt and DMA
            GPIO::State Dir;                        // Define the GPIO state for DIRECTION pin
            GPIO::State nRst;                       // Define the GPIO state for Reset pin
            GPIO::State Micr[StpMaxMicroStepPins];  // Define the GPIO state(s) for Microstep pins
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
        GPIO        *nResePin;      // Provide a pointer to the GPIO class, which is connected to
                                    // the RESET pin -  "LOW" = RESET
        GPIO        *Dirction;      // Provide a pointer to the GPIO class, which is connected to
                                    // the DIR pin
        GPIO        *MicrStep;      // Provide a pointer to the GPIO class(es), which are
                                    // connected to the MS/MicroStep pins
        uint8_t     MicStpSiz;      // Number of pins used for Microstep

    public:     // Following are shadow values which are the current active state of the Stepper
                // driver
        Form            FormArry[StpBuffSize];  // Array to contain the Stepper request forms
        GenBuffer<Form> FormQueue;              // GenBuffer for the Forms

        uint16_t    ShdPfrl[2];     // Profile to use for STEP pulse
        CountPanel  ShdCountConf;   // Shadow (active) Count configuration
        Form        ShdForm;        // Shadow entry of the controlling signals which need to be
                                    // synced with the rising edge of STEP
    public:
        int32_t    FullRev;         // Defines the maximum number of steps per revolution of
                                    // stepper (needs to include the driver smallest step rate,
                                    // i.e. Motor has 200 poles, driver can allow 1/16 step per
                                    // pulse, therefore this would be 200 x 16 = 3,200
        volatile int32_t    calcPos;    // Current calculated position of the stepper pole

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
    protected:
        void popGenParam(void);         // Populate generic parameters for the class

    public:
        Stepper(void);                  // Basic constructor for Stepper class
#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    private:
        TIM_HandleTypeDef   *STEP;      // Hardware controlled TIMER configured in "Output
                                        // Compare" mode, toggling the STEP GPIO pin
        DMA_HandleTypeDef   *DMA_stp;   // DMA linked to the specific TIM + channel for the
                                        // "Output Compare" - Output
        uint32_t            OtpCopChnl; // Channel for the TIM "Output Compare" signal (OUTPUT)
                                        // This contains the Channel state shifted up by the
                                        // Output channel

        HrdSetup    HrdwreCfg;          // Defined at class construction time

    public:
        void create(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                    uint32_t CountChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                    int32_t FullRev, HrdSetup Config);

        Stepper(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                uint32_t CountChannel, GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
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
//==================================================================================================

#else
//==================================================================================================

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
    void SetShadowGPIO(void);           // Set the GPIO pins as per shadow.
    void InterruptSetup(void);          // Enable the STEP pulse generator interrupt, if the
                                        // system is currently "DISABLED". Otherwise leave in
                                        // buffer
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

    void IRQUPHandler(void);            // Interrupt handler to be placed within the "TIMx_UP"
                                        // interrupt request, or interrupt triggered at base timer
                                        // overflow
    void IRQCounterCCHandler(void);     // Interrupt handler to be placed within the "TIMx_CC"
                                        // interrupt request, or interrupt triggered due to
                                        // Output Compare for Counter controller.
    virtual ~Stepper();
};

#endif /* STEPPER_H_ */
