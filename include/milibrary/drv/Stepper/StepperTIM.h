/**************************************************************************************************
 * @file        StepperTIM.h
 * @author      Thomas
 * @brief       Header file for the Stepper Driver Class handle (TIM Interrupt)
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
 * This class combines together "StepperCore", and the "DMAPeriph" classes, introducing the TIM
 * interrupt control capabilities.
 * The changes that this class introduces relative to the "StepperCORE" is:
 *      Public visible function -
 *             V".handleIRQ"        - Function to be placed in the TIM Update Interrupt, will
 *                                    handle the interrupt in all (stepper) modes - updating the
 *                                    estimated stepper position, etc.
 *
 *      Protected function(s) -
 *             V".startInterrupt"   - Will configure the DMA, TIM, and interrupts for any new
 *                                    movement requests.
 *             V".disableMotor"     - Will Disable the TIM interrupts
 *
 *  [#] How the TIM class is configured
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      This class so as to make sole use of the connected TIM to the Stepper IC; varying the
 *      number of "counts" before the count overflow/update event occurs. There is then a single
 *      array of 2 variables (_shd_profile_) which is connected to the DMA {to be configured in
 *      circular mode}, this array will have the same two values throughout the function this is
 *      to generate the pulse shape.
 *      The varying timer, connected to the DMA, will ensure that the pulse coming out of the
 *      compare output channel is the specified frequency.
 *
 *  [#] How Stepper PULSE is generated, and managed
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      As stated within the "StepperCore" this class is able to support 2 movement modes:
 *      "Position" and "Velocity".
 *
 *      If the class is in "Position" mode, then the Overflow interrupt will only count the number
 *      of pulses (against the requested amount), as well as calculating the estimated position
 *      based upon the microstep (gear) specified.
 *      Once the required number of pulses has been achieved, then the interrupt will be disabled
 *      (calling ".disableMotor"), and check for any new movement requests.
 *
 *      If in the "Velocity" mode, the only difference to "Position" mode is that the interrupt
 *      will be continually checking for a new movement request.
 *      Once it detects a new movement request, if the request is a new "Velocity" mode, then this
 *      frequency and GPIO values are applied straight away. However if the next movement is
 *      "Position", then the interrupt will be disabled (again via ".disableMotor"), and this new
 *      request loaded.
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
 * Count    0                             |     1                             |     2
 *        ________________________________       _____________________________       ___________
 * Auxi   ________________________________X-----X_____________________________X-----X___________
 *
 * Pulse configured such that first toggle occurs when TIMER = 50 (kdefault_profile_delay), and
 * second toggle occurs when it equals 55 (DMA manages this, kdefault_profile_delay +
 * kdefault_profile_width).
 *
 * OvFlo    - Interrupt will be called at the Update Event (or Counter overflow), and will count
 *            number pulses, and setup the GPIO (Auxi in diagram above)
 * 
 *************************************************************************************************/
#ifndef STEPPERTIM_H_
#define STEPPERTIM_H_

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
#include FilInd_StpCOREHD               // Header for Stepper (Core Header)
#include FilInd_GPIO___HD               // Allow use of GPIO class
#include FilInd_DMAPe__HD               // Allow use of the DMA class

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class StepperTIM : public StepperCore, public DMAPeriph {
protected:
     static const uint8_t   kdefault_profile_delay      = 50;
     /* This value defines the count value of the Timer where the Output Compare signal will
      * toggle initially (FALSE->TRUE). Due to the Update Event interrupt occuring before this,
      * there needs to be sufficient time between this interrupt and the pulse being set - so as to
      * allow the auxiliary GPIO signals to be configured correctly prior to the pulse.
      * 50 counts has been selected for this
      */

/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "StepperTIM::" followed by the type.
 *************************************************************************************************/
// Capture in the CORE version

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        uint16_t    _shd_profile_[2];   // Profile to use for STEP pulse

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    public:
        StepperTIM(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                   GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                   int32_t FullRev, HrdSetup Config);
        /******************************************************************************************
         * Inputs required for StepperCore, so see this for explainations of what the inputs need
         * to be
         *****************************************************************************************/

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================

#else
//=================================================================================================

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "StepperTIM" class, which are generic; this
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
    void startInterrupt(void);          // Enable the STEP pulse generator interrupt, if the
                                        // system is currently "DISABLED". Otherwise leave in
                                        // buffer
    void disableMotor(void);            // Function to be called when transitioning from one
                                        // position demand, to another
public:     /**************************************************************************************
             * ==  PUBLIC   == >>>        CLASS INTERFACE FUNCTIONS        <<<
             *   -----------
             *  Visible functions used to request any new position or velocity of the stepper
             *  motor.
             *  Functions are designed in interrupt mode exclusively.
             *************************************************************************************/
    void handleIRQ(void);               // Interrupt Handle for Stepper Device (based on TIM
                                        // transmit complete)

    virtual ~StepperTIM();
};

#endif /* STEPPERTIM_H_ */
