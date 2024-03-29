/**************************************************************************************************
 * @file        StepperDMA.h
 * @author      Thomas
 * @brief       Header file for the Stepper Driver Class handle (DMA Interrupt)
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
 * This class combines together "StepperCore", and the "DMAPeriph" classes, introducing the DMA
 * interrupt control capabilities.
 * The changes that this class introduces relative to the "StepperCORE" is:
 *      Public visible function -
 *             V".handleIRQ"        - Function to be placed in the DMA Interrupt, will handle the
 *                                    interrupt in all (stepper) modes - updating the estimated
 *                                    stepper position, etc.
 *
 *      Protected function(s) -
 *              ".updatePulseDMA"   - Will calculate the 'count' value to be stored within the
 *                                    linked DMA array, so as to get the required step frequency.
 *                                    Will also disabled/re-enabled the DMA to ensure update is
 *                                    captured.
 *
 *             V".startInterrupt"   - Will configure the DMA, and interrupts for any new movement
 *                                    requests.
 *             V".disableMotor"     - Will Disable the DMA interrupts
 *
 *  [#] How the TIM class is configured
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      This class so as to allow the TIM to be used by multiple Steppers, the TIM will be fixed
 *      to overflow once full (i.e. 0xFFFF), the individual DMAs connected to the Output Compare
 *      channels will be configured to interrupt onces the array has been transferred.
 *      At which point the software will calculate what the values of the linked DMA array need to
 *      be so as to achieved the desired pulse frequency.
 *        note - the size of the linked DMA array is configurable at compilation time, however
 *        analysis has shown that the best value is 10 (so as to gain the best timing benefit).
 *
 *  [#] How Stepper PULSE is generated, and managed
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *      As stated within the "StepperCore" this class is able to support 2 movement modes:
 *      "Position" and "Velocity".
 *
 *      If the class is in "Position" mode, then the DMA transmit complete intrrupt will only count
 *      the number of pusles (against the requested amount), as well as calculating the estimated
 *      position based upon the microstep (gear) specified.
 *      Once the required number of pulses has been achieved, then the interrupt will be disabled
 *      (calling ".disableMotor"), and check for any new movement requests.
 *
 *      If in the "Velocity" mode, the only difference to "Position" mode is that the interrupt
 *      will be continually checking for a new movement request.
 *      Once it detects a new movement request, if the request is a new "Velocity" mode, then this
 *      frequency and GPIO values are applied straight away (and the linked DMA updated with new
 *      pulse rate). However if the next movement is "Position", then the interrupt will be
 *      disabled (again via ".disableMotor"), and this new request loaded.
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
 * DMATC  _______|___________________________________|___________________________________|______
 *
 * Count    0    |     1                             |     2                             |     3
 *        _______       _____________________________       _____________________________
 * Auxi   _______X-----X_____________________________X-----X_____________________________X-----X
 *
 * DMATC    - Interrupt called at DMA transmit complete, to count, configure GPIO and re-calculate
 *            DMA Array values for requested frequency.
 * 
 *************************************************************************************************/
#ifndef STEPPERDMA_H_
#define STEPPERDMA_H_

#include "FileIndex.h"
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
#include FilInd_StpCOREHD               // Header for Stepper (Core Header)
#include FilInd_GPIO___HD               // Allow use of GPIO class
#include FilInd_DMAPe__HD               // Allow use of the DMA class

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class StepperDMA : public StepperCore, public DMAPeriph {
protected:
     static const uint8_t   kprofile_array_pulse_depth  = 10;

     static const uint16_t  kcount_limit                = 0xFFFF;
     /* Pulse depth, is the number of pulses that will be stored within the linked DMA array, value
      * has been determined from analysis captured within the "analysis" folder of miLibrary.
      * Values as per commit - 'df6760d' (wip-#5-stepper-improvements branch)
      *
      * This value is the maximum 'count' that the TIM will achieve before overflowing (or
      * triggering an update event.
      * Used to calculate what the linked DMA array values need to be to get the required pulse
      * frequency.
      */

/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "StepperDMA::" followed by the type.
 *************************************************************************************************/
// Capture in the CORE version

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        uint16_t    _shd_profile_[kprofile_array_pulse_depth * 2];
            // Profile to use for STEP pulse
        uint8_t     _pulse_run_;        // Number of pulses that will occur in a single DMA
                                        // transfer

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    public:
        StepperDMA(TIM_HandleTypeDef *STEP_TIM, DMA_HandleTypeDef *STEP_DMA, uint32_t OCChannel,
                   GPIO *nReset, GPIO *DIR, GPIO *MicStp, uint8_t McrStp,
                   int32_t FullRev, HrdSetup Config);
        /******************************************************************************************
         * Inputs required for StepperCore, so see this for explainations of what the inputs need
         * to be
         *****************************************************************************************/

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================

#else
//=================================================================================================

#endif

    virtual ~StepperDMA();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "StepperDMA" class, which are generic; this
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
    void updatePulseDMA(uint16_t prevposition); // Calculate the next pulse position, and
                                                // implement in array
    void disableMotor(void);            // Function to be called when transitioning from one
                                        // position demand, to another
public:     /**************************************************************************************
             * ==  PUBLIC   == >>>        CLASS INTERFACE FUNCTIONS        <<<
             *   -----------
             *  Visible functions used to request any new position or velocity of the stepper
             *  motor.
             *  Functions are designed in interrupt mode exclusively.
             *************************************************************************************/
    void handleIRQ(void);               // Interrupt Handle for Stepper Device (based on DMA
                                        // transmit complete)
};

#endif /* STEPPERDMA_H_ */
