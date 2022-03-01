/**************************************************************************************************
 * @file        LnxCond.h
 * @author      Thomas
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
 *      To retrieve and calculate the temperature/load call "updateStatus", this will return the
 *      status of the read - if no failures will return 'kNone' -> see "DevFlt" below.
 *      Can then call the following:
 *          ".temperature"          - For the temperature of CPU
 *          ".cpu_load[]"           - Load on the CPU, this is an array with size "LNX_NUM_CORES"
 *                                    + 1 with the first entry the total, and subsequent entries
 *                                    individual cores.
 *          ".cpu_raw_load[]"       - Array containing each data point of the CPU, is the same
 *                                    size as "cpu_load" however each contain 10 additional
 *                                    entries for each CPU count.
 *                                      cpu_raw_load[0][0] = CPU (Total) - Time in user mode
 *                                      cpu_raw_load[1][0] = CPU (  1  ) - Time in user mode
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef LNXCOND_H_
#define LNXCOND_H_

// C System Header(s)
// ------------------
#include <fstream>      // std::ifstream
#include <sstream>      // std::stringstream
#include <string>       // std::string

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// As currently have only 1 Embedded Linux Device, this call will only work if the project has
// been configured for Raspberry Pi
#error "Unrecognised target device"

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// As currently have only 1 Embedded Linux Device, this call will only work if the project has
// been configured for Raspberry Pi
#error "Unrecognised target device"

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
// None

//=================================================================================================

// Defines specific within this class
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

// Types used within this class
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  > CPU Status defines
#define LNX_NUM_CPU_STATES  10        // Number of entries within CPU status file

#ifndef LNX_NUM_CORES                 // If the number of CPU cores is not defined
#define LNX_NUM_CORES       4         // Define the number as 4 (excluding the total)
#endif

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

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class LnxCond {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "LnxCond::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {       // Fault TYpe of the class (internal enumerate)
        kNone               = 0x00,     // Normal Operation
        kTemperature_Open   = 0x01,     // Fault with the CPU Temperature file open
        kTemperature_Read   = 0x02,     // Fault with CPU Temperature file read

        kCPU_Load_Open      = 0x03,     // Fault with CPU Load file open
        kCPU_Load_Read      = 0x04,     // Fault with CPU Load file read
        kCPU_Load_Convert   = 0x05,     // Fault with CPU Load file conversion

        kInitialised        = 0xFF      // Initialisation
    };

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        std::string _temperature_file_;     // Location for where CPU temperature file is located
        std::string _cpu_file_;             // Location for where CPU load file is located

        uint32_t    _previous_cpu_raw_load_[LNX_NUM_CORES + 1][LNX_NUM_CPU_STATES];
                    // CPU entries from previous read
        float       _check_frequency_;      // Frequency of checking
        uint8_t     _first_pass_;           // Variable for determining if the class has gone
                                            // through initial run through

    public:
        float       temperature;            // Calculated temperature (celsius) from last
                                            // "UpdateStatus"
        float       cpu_load[LNX_NUM_CORES + 1];
                    // Calculated load on CPU - for each core defined (+1 for total)
        DevFlt      flt;                    // Store the FaultCode from previous "UpdateStatus"
        uint32_t    cpu_raw_load[LNX_NUM_CORES+1][LNX_NUM_CPU_STATES];
                    // CPU entries at current read

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
    LnxCond();                          // Default class initialise
    LnxCond(float Frequency);           // Default class + set Check Frequency
    /**************************************************************************************************
     * LnxCond is an overloaded function, so therefore has multiple calling conditions
     *  The first   is the "default", where no arguments are passed to it - it will then setup default
     *              parameters
     *  The second  is when the frequency is provided to the class. It will again setup default
     *              parameters, however will set the frequency to value provided
     *************************************************************************************************/

    virtual ~LnxCond();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "LnxCond" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>     DIRECT HARDWARE READING FUNCTIONS     <<<
             *   -----------
             *  Functions allow direct access to the hardware/base functions of this class. Are
             *  not visible unless "friend" or "child"/inherited
             *************************************************************************************/
        DevFlt      convertCPUText(uint32_t CPUData[], std::string CPUString);
                    // Convert the read CPU status string into array entries
        void        calculateCPULoad(void); // Calculate the CPU load
        void        updateCPUHistory(void); // Update the CPU historic array

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to update the class internals for the OS CPU load and
             *  temperature.
             *************************************************************************************/
        DevFlt updateStatus(void);         // Update class parameters to latest conditions
};

#endif /* LNXCOND_H_ */
