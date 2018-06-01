/**************************************************************************************************
 * @file        LnxCond.h
 * @author      thomas
 * @version     V0.2
 * @date        1 June 2018
 * @brief       Header file for the Linux Embedded Device Operating Conditions
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Linux class, which will retrieve the CPU conditions of the Linux filesystem that this script is
 * run on.
 * Will retrieve the CPU temperature (does not support multiple CPUs temperature), and will get the
 * load of the CPU (the number of cores within the processor needs to be defined for the script
 * via - "LNX_NUM_CORES".
 * Use of class
 *      Initial call, doesn't require any additional parameters
 *          Optional -> The frequency of the checking of the files can be provided, although this
 *                      is not used for any calculations
 *
 *      To retrieve and calculate the temperature/load call "UpdateStatus", this will return the
 *      status of the read - if no failures will return 'NoFault' -> see "_LnxFlt" below.
 *      Can then call the following:
 *          ".Temp"     = For the temperature of CPU
 *          ".CurLoad[]"= Load on the CPU, this is an array with size "LNX_NUM_CORES" + 1
 *                        with the first entry the total, and subsequent entries individual cores.
 *          ".CurCPU[]" = Array containing each data point of the CPU, is the same size as
 *                        "CurLoad" however each contain 10 additional entries for each CPU count.
 *                        CurCPU[0][0] = CPU (Total) - Time in user mode
 *                        CurCPU[1][0] = CPU (  1  ) - Time in user mode
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef LNXCOND_LNXCOND_H_
#define LNXCOND_LNXCOND_H_

#include <fstream>      // std::ifstream
#include <sstream>      // std::stringstream
#include <string>       // std::string

#if defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// As currently have only 1 Embedded Linux Device, this class will only work if the project has
// been configured for RaspberryPi
#else
//==================================================================================================
// Otherwise it is an unrecognised device
#error "Unrecognised target device"

#endif

// Defines specific within this class
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef enum {
    LnxCond_NoFault = 0,            // No fault with Linux update
    LnxCond_TemperatureOpen = 1,    // Fault with CPU Temperature file open
    LnxCond_TemperatureRead = 2,    // Fault with CPU Temperature file read

    LnxCond_CPULoadOpen     = 3,    // Fault with CPU Load file open
    LnxCond_CPULoadRead     = 4,    // Fault with CPU Load file read
    LnxCond_CPULoadConvert  = 5,    // Fault with CPU Load file conversion


    LnxCond_Initialised = -1        // If initailised, this flag is set
} _LnxFlt;

// Types used within this class
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  > CPU Status defines
#define LNX_NUM_CPU_STATES  10        // Number of entries within CPU status file

#ifndef LNX_NUM_CORES                 // If the number of CPU cores is not defined
#define LNX_NUM_CORES       4         // Define the number as 4 (excluding the total)
#endif

//  > Class Mode defines
#define LNX_FIRST_PASS          0x01  // Flag indicating that first pass has been done (classmode)

//  > Short-cuts for calculating ActiveTime and IdleTime
#define LNX_ActiveTime(array, core)     array[core][0] + \
                                        array[core][1] + \
                                        array[core][2] + \
                                        array[core][5] + \
                                        array[core][6] + \
                                        array[core][7] + \
                                        array[core][8] + \
                                        array[core][9]
/**************************************************************************************************
 * Shortcut for calculating ActiveTime of CPU:
 *  column 1    = Time in User Mode
 *  column 2    = Time in User Mode (low priority)
 *  column 3    = Time in System Mode
 *  column 6    = Time Servicing Hardware Interrupts
 *  Column 7    = Time Servicing Software Interrupts
 *  Column 8    = Time in other Operating Systems (when in virtualised environment)
 *  Column 9    = Time spent running virtual CPU for guest OS
 *  Column 10   = Time spent running low priority virtual CPU for guest OS
 *************************************************************************************************/
#define LNX_IdleTime(array, core)       array[core][3] + \
                                        array[core][4]
/**************************************************************************************************
 * Shortcut for calculating IdleTime of CPU
 *  column 4    = Time spent in idle task
 *  column 5    = Time waiting for I/O to complete
 *************************************************************************************************/

class LnxCond {
    private:
        std::string TemperatureFile;        // Location for where CPU temperature file is located
        std::string CPUFile;                // Location for where CPU load file is located
        uint32_t    PrevCPU[LNX_NUM_CORES + 1][LNX_NUM_CPU_STATES];
                    // CPU entries from previous read
        float       CheckFreq;              // Frequency of checking
        int8_t      classmode;              // Variable for storing parameters on the class
                                            // i.e. FirstPass state


        void        InitialSetup(void);     // Hidden function, which defaults all parameters
        _LnxFlt     ConvertCPUText(uint32_t CPUData[], std::string CPUString);
                    // Convert the read CPU status string into array entries
        void        CalculateCPULoad(void); // Calculate the CPU load
        void        UpdateCPUHistory(void); // Update the CPU historic array

    public:
        float       Temp;                   // Calculated temperature (celsius) from last
                                            // "UpdateStatus"
        float       CurLoad[LNX_NUM_CORES + 1];
                    // Calculated load on CPU - for each core defined (+1 for total)
        _LnxFlt     FaultCode;              // Store the FaultCode from previous "UpdateStatus"
        uint32_t    CurCPU[LNX_NUM_CORES+1][LNX_NUM_CPU_STATES];
                    // CPU entries at current read

/**************************************************************************************************
 * LnxCond is an overloaded function, so therefore has multiple calling conditions
 *  The first   is the "default", where no arguments are passed to it - it will then setup default
 *              parameters
 *  The second  is when the frequency is provided to the class. It will again setup default
 *              parameters, however will set the frequency to value provided
 *************************************************************************************************/
        LnxCond();                          // Default class initialise
        LnxCond(float Frequency);           // Default class + set Check Frequency

        _LnxFlt UpdateStatus(void);         // Update class parameters to latest conditions

        virtual ~LnxCond();
};

#endif /* LNXCOND_LNXCOND_H_ */
