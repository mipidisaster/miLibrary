/**************************************************************************************************
 * @file        LnxCond.cpp
 * @author      Thomas
 * @brief       Source file for the Linux Embedded Device Operating Conditions
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_LnxCondHD               // Header for Linus Condition

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

void LnxCond::popGenParam() {
/**************************************************************************************************
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    uint8_t i, j   = 0;             // Variable for looping
    _temperature_file_  = "/sys/class/thermal/thermal_zone0/temp";
                                        // Default location for CPU Temp
    _cpu_file_          = "/proc/stat";
                                        // Default location for CPU status
    _check_frequency_   = 1;            // Default for read rate

    temperature     = -999;         // Setup temperature to initially be very low
    _first_pass_    = 1;
    flt             = DevFlt::kInitialised;

    for (j = 0; j != (LNX_NUM_CORES + 1); j++) {    // Loop through the top level array entries
        cpu_load[j]  = 0.00;                        // Setup load to 0.00%

        for (i = 0; i != LNX_NUM_CPU_STATES; i++) { // Loop through lower level array entries
            _previous_cpu_raw_load_[j][i]   = 0;
            cpu_raw_load[j][i]              = 1;
        }
    }
}

LnxCond::LnxCond() {
/**************************************************************************************************
 * Calling condition for Linux Embedded Condition class.
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    popGenParam();            // Call hidden function defaulting all parameters
}

LnxCond::LnxCond(float Frequency) {
/**************************************************************************************************
 * Calling condition for Linux Embedded Condition class.
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    popGenParam();                  // Call hidden function defaulting all parameters
    _check_frequency_ = Frequency;  // Set the Check Frequency as per input
}

LnxCond::DevFlt LnxCond::convertCPUText(uint32_t CPUData[], std::string CPUString) {
/**************************************************************************************************
 * Function will read the input string - "CPUString", and convert the text into entries within
 * array - CPUData[].
 * It is only expecting to see 10 columns of data, and will ignore the first entry as this will
 * be "cpu", "cpu0", etc.
 *************************************************************************************************/
    uint8_t i = 0;              // Variable to keep trace of entries converted
    std::string subline;        // Variable to store temporary line
    std::stringstream stream;   // Create class "stringstream"

    if (CPUString.compare(0, 3, "cpu") == 0) {  // Check beginning of string for "cpu"
        // If equal to "0", then they are equal, otherwise not exactly equal
        stream.str (CPUString);                 // Provide stream with input string
        // As we know that the first 3 characters are "cpu", update character point to start at
        // character 5
        stream.seekg(5, std::ios::beg);

        while(std::getline(stream, subline, ' ')) { // Cycle through line, and capture character's
                                                    // between " " (spaces)
            if (i >= LNX_NUM_CPU_STATES)            // If the number of loops exceeds defined size
                return (flt = DevFlt::kCPU_Load_Convert);   // return fault

            CPUData[i++] = std::stoi(subline, nullptr);
        }

    } else
        return (flt = DevFlt::kCPU_Load_Convert);   // If the start of the string isn't "cpu" then
                                                    // layout is unexpected, and fault is to be set

    return (flt = DevFlt::kNone);   // If have made it this far then, data has been converted
                                    // without error
}

void LnxCond::updateCPUHistory(void) {
/**************************************************************************************************
 * Function will loop through array entries of the "cpu_raw_load", and transfer across to
 * "_previous_cpu_raw_load_"
 *************************************************************************************************/
    uint8_t cores;          // Variable for looping the cores within array
    uint8_t datapoints;     // Variable for looping through data points within array

    for (cores = 0; cores != (LNX_NUM_CORES + 1); cores++)
        for (datapoints = 0; datapoints != LNX_NUM_CPU_STATES; datapoints++) {
            _previous_cpu_raw_load_[cores][datapoints] = cpu_raw_load[cores][datapoints];
            // Loop through data, and transfer to historical array
    }
}

void LnxCond::calculateCPULoad(void) {
/**************************************************************************************************
 * Based upon provided values, calculate the CPU load of each cores within class
 * Calculated, based upon determining the IDLE and ACTIVE time of each core, at previous and
 * current sample
 * The load is then determined by:
 *
 *      Cur.ActiveTime - Prev.ActiveTime = ActiveDiff
 *      Cur.IdleTime   - Prev.IdleTime   = IdleDiff
 *
 *      Load = ActiveDiff / (ActiveDiff + IdleDiff)
 *************************************************************************************************/
    uint32_t current_active_time    = 0;    // Calculated Current Active Time
    uint32_t current_idle_time      = 0;    // Calculated Current Idle Time

    uint32_t previous_active_time   = 0;    // Calculated Previous Active Time
    uint32_t previous_idle_time     = 0;    // Calculated Previous Idle Time

    uint32_t active_difference      = 0;    // Difference between Active Times
    uint32_t idle_diff              = 0;    // Difference between Idle Times

    uint8_t cores = 0;              // Variable to loop through cores

    for (cores = 0; cores != (LNX_NUM_CORES + 1); cores++) {
        current_idle_time    = LNX_IdleTime(cpu_raw_load, cores);
        current_active_time  = LNX_ActiveTime(cpu_raw_load, cores);

        previous_idle_time   = LNX_IdleTime(_previous_cpu_raw_load_, cores);
        previous_active_time = LNX_ActiveTime(_previous_cpu_raw_load_, cores);

        active_difference = current_active_time - previous_active_time;     // Diff Active Times
        idle_diff   = current_idle_time - previous_idle_time;               // Diff Idle Times

        cpu_load[cores] = ((float)active_difference) / ((float)(active_difference + idle_diff));
            // Calculate the load for CPU core
    }
}

LnxCond::DevFlt LnxCond::updateStatus(void) {
/**************************************************************************************************
 * Function to update the Class's internal parameters to the latest conditions of the Linux
 * Embedded Device
 * If any failure has been detected during reading of files, the function will return a fault
 * code, as per the enumerate type.
 *************************************************************************************************/
    std::ifstream emb_dev_file;     // Stream used to read parameters within file structure
    std::string line;               // String variable for capturing single line from file up to
                                    // "\n"
    int temp_integer;               // Temporary Integer for function

    // First check is to retrieve the CPU temperature of Linux Embedded Device
    emb_dev_file.open(_temperature_file_);              // Open file
    if (!emb_dev_file.is_open()) {                      // If unable to open file
        return (flt = DevFlt::kTemperature_Open);       // Update Fault code
    }

    else {  // If read is successful then
        if (std::getline(emb_dev_file, line)) { // Read the first (and only) line in Temperature
                                                // file
            temp_integer = std::stoi(line, nullptr);    // Convert number in file to integer
            temperature = ((float)temp_integer) / 1000; // Transform scaled number into floating
                                                        // point
        }
        else {                                          // If read is unsuccessful then:
            return (flt = DevFlt::kTemperature_Read);   // Update Fault code
        }
    }

    emb_dev_file.close();         // Close file

    // Second check is to retrieve and calculate the CPU load
    // Now this requires 2 points to determine how the load has changed relative to the 2 points
    // So if this is the first pass, then need to capture initial data point. Then on second+
    // runs can determine the load
    uint8_t j;                      // Variables used for looping with arrays

    emb_dev_file.open(_cpu_file_);                      // Open file
    if (!emb_dev_file.is_open()) {                      // If unable to open file
        return (flt = DevFlt::kCPU_Load_Open);          // Update Fault code
    }

    else {  // If read is successful then
            // Update the current CPU array to the latest data
        for (j = 0; j != (LNX_NUM_CORES + 1); j++) {// Loop through the top level array entries
            if (std::getline(emb_dev_file, line)) {   // Read the line, and then convert
                flt = convertCPUText(cpu_raw_load[j], line);
                    // Convert line into array entries
                if (flt != DevFlt::kNone)
                    return(flt);                        // Return fault
            }
            else {                                      // If read is unsuccessful then:
                return (flt = DevFlt::kCPU_Load_Read);  // Update Fault code
            }
        }

        // Now the array "cpu_raw_load" has been updated with the latest status of the CPU
        // Check if this has been the first pass, if it has, then cannot calculate loads
        if (_first_pass_ == 1) {
            updateCPUHistory();         // Update Historical CPU array
        }
        else {
            calculateCPULoad();         // Calculate the CPU Load
        }

        _first_pass_ = 0;
    }

    emb_dev_file.close();         // Close file

    return (flt = DevFlt::kNone);   // If have made it this far, then function has completed
                                    // successfully
}

LnxCond::~LnxCond()
{
    // TODO Auto-generated destructor stub
}

