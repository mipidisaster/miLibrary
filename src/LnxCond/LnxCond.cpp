/**************************************************************************************************
 * @file        LnxCond.cpp
 * @author      Thomas
 * @brief       Source file for the Linux Embedded Device Operating Conditions
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "LnxCond/LnxCond.h"

void LnxCond::InitialSetup() {
/**************************************************************************************************
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    uint8_t i, j   = 0;             // Variable for looping
    this->TemperatureFile = "/sys/class/thermal/thermal_zone0/temp";
                                        // Default location for CPU Temp
    this->CPUFile         = "/proc/stat";
                                        // Default location for CPU status
    this->CheckFreq       = 1;          // Default for read rate

    this->Temp            = -999;       // Setup temperature to initially be very low
    this->classmode       = LNX_FIRST_PASS;         // Default mode set to "FIRST_PASS"
    this->FaultCode       = LnxCond_Initialised;    // Default FaultCode to initialised

    for (j = 0; j != (LNX_NUM_CORES + 1); j++) {    // Loop through the top level array entries
        this->CurLoad[j]  = 0.00;                   // Setup load to 0.00%

        for (i = 0; i != LNX_NUM_CPU_STATES; i++) { // Loop through lower level array entries
            this->PrevCPU[j][i]     = 0;            // and clear them all
            this->CurCPU[j][i]      = 1;            // and clear them all
        }
    }
}

LnxCond::LnxCond() {
/**************************************************************************************************
 * Calling condition for Linux Embedded Condition class.
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    this->InitialSetup();           // Call hidden function defaulting all parameters
}

LnxCond::LnxCond(float Frequency) {
/**************************************************************************************************
 * Calling condition for Linux Embedded Condition class.
 * When invoking this, it will populate the class with default parameters for locations of files
 * within the Linux operating system
 *************************************************************************************************/
    this->InitialSetup();           // Call hidden function defaulting all parameters
    this->CheckFreq = Frequency;    // Set the Check Frequency as per input
}

_LnxFlt LnxCond::ConvertCPUText(uint32_t CPUData[], std::string CPUString) {
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
                return (LnxCond_CPULoadConvert);    // return fault

            CPUData[i++] = std::stoi(subline, nullptr);
        }

    } else
        return (LnxCond_CPULoadConvert);        // If the start of the string isn't "cpu" then
                                                // layout is unexpected, and fault is to be set

    return (LnxCond_NoFault);   // If have made it this far then, data has been converted without
                                // error
}

void LnxCond::UpdateCPUHistory(void) {
/**************************************************************************************************
 * Function will loop through array entries of the "CurCPU", and transfer across to "PrevCPU"
 *************************************************************************************************/
    uint8_t cores;          // Variable for looping the cores within array
    uint8_t datapoints;     // Variable for looping through data points within array

    for (cores = 0; cores != (LNX_NUM_CORES + 1); cores++)
        for (datapoints = 0; datapoints != LNX_NUM_CPU_STATES; datapoints++) {
            this->PrevCPU[cores][datapoints] = this->CurCPU[cores][datapoints];
            // Loop through data, and transfer to historical array
    }
}

void LnxCond::CalculateCPULoad(void) {
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
    uint32_t Cur_ActiveTime = 0;    // Calculated Current Active Time
    uint32_t Cur_IdleTime   = 0;    // Calculated Current Idle Time

    uint32_t Prev_ActiveTime = 0;   // Calculated Previous Active Time
    uint32_t Prev_IdleTime = 0;     // Calculated Previous Idle Time

    uint32_t ActiveDiff = 0;        // Difference between Active Times
    uint32_t IdleDiff   = 0;        // Difference between Idle Times

    uint8_t cores = 0;              // Variable to loop through cores

    for (cores = 0; cores != (LNX_NUM_CORES + 1); cores++) {
        Cur_IdleTime    = LNX_IdleTime(this->CurCPU, cores);    // Calculate Idle Time (Current)
        Cur_ActiveTime  = LNX_ActiveTime(this->CurCPU, cores);  // Calculate Active Time (Current)

        Prev_IdleTime   = LNX_IdleTime(this->PrevCPU, cores);   // Calculate Idle Time (Prev)
        Prev_ActiveTime = LNX_ActiveTime(this->PrevCPU, cores); // Calculate Active Time (Prev)

        ActiveDiff = Cur_ActiveTime - Prev_ActiveTime;          // Diff Active Times
        IdleDiff   = Cur_IdleTime - Prev_IdleTime;              // Diff Idle Times

        this->CurLoad[cores] = ((float)ActiveDiff) / ((float)(ActiveDiff + IdleDiff));
            // Calculate the load for CPU core
    }
}

_LnxFlt LnxCond::UpdateStatus(void) {
/**************************************************************************************************
 * Function to update the Class's internal parameters to the latest conditions of the Linux
 * Embedded Device
 * If any failure has been detected during reading of files, the function will return a fault
 * code, as per the enumerate type.
 *************************************************************************************************/
    std::ifstream EmbDevfile;       // Stream used to read parameters within file structure
    std::string line;               // String variable for capturing single line from file up to
                                    // "\n"
    int TempInt;                    // Temporary Integer for function

    // First check is to retrieve the CPU temperature of Linux Embedded Device
    EmbDevfile.open(this->TemperatureFile);     // Open file
    if (!EmbDevfile.is_open()) {                // If unable to open file
        this->FaultCode = LnxCond_TemperatureOpen;  // Update Fault Code
        return (this->FaultCode);                   // Return fault
    }
    else {  // If read is successful then
        if (std::getline(EmbDevfile, line)) {   // Read the first (and only) line in Temperature
                                                // file
            TempInt = std::stoi(line, nullptr);     // Convert number in file to integer
            this->Temp = ((float)TempInt) / 1000;   // Transform scaled number into floating point
        }
        else {                                  // If read is unsuccessful then:
            this->FaultCode = LnxCond_TemperatureRead;  // Update Fault Code
            return (this->FaultCode);                   // Return fault
        }
    }

    EmbDevfile.close();         // Close file

    // Second check is to retrieve and calculate the CPU load
    // Now this requires 2 points to determine how the load has changed relative to the 2 points
    // So if this is the first pass, then need to capture initial data point. Then on second+
    // runs can determine the load
    uint8_t j;                      // Variables used for looping with arrays

    EmbDevfile.open(this->CPUFile);         // Open file
    if (!EmbDevfile.is_open()) {            // If unable to open file
        this->FaultCode = LnxCond_CPULoadOpen;  // Update Fault Code
        return (this->FaultCode);               // Return fault
    }
    else {  // If read is successful then
            // Update the current CPU array to the latest data
        for (j = 0; j != (LNX_NUM_CORES + 1); j++) {// Loop through the top level array entries
            if (std::getline(EmbDevfile, line)) {   // Read the line, and then convert
                this->FaultCode = this->ConvertCPUText(this->CurCPU[j], line);
                    // Convert line into array entries
                if (this->FaultCode != LnxCond_NoFault)
                    return(this->FaultCode);        // Return fault
            }
            else {                                  // If read is unsuccessful then:
                this->FaultCode = LnxCond_CPULoadRead;  // Update Fault Code
                return (this->FaultCode);               // Return fault
            }
        }

        // Now the array "CurCPU" has been updated with the latest status of the CPU
        // Check if this has been the first pass, if it has, then cannot calculate loads
        if ((this->classmode & LNX_FIRST_PASS) != LNX_FIRST_PASS) {     // If not First Pass
            this->CalculateCPULoad();       // Calculate the CPU Load

        } else                                  // If First Pass
            this->classmode &= ~LNX_FIRST_PASS; // Clear flag
        // Update Historical CPU array
        this->UpdateCPUHistory();
    }

    EmbDevfile.close();         // Close file

    this->FaultCode = LnxCond_NoFault;  // Update Fault Code
    return (this->FaultCode);           // If have made it this far, then function has completed
                                        // successfully return safe code
}

LnxCond::~LnxCond()
{
    // TODO Auto-generated destructor stub
}

