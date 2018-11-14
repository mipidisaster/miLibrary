/**************************************************************************************************
 * @file        GenBuffer.cpp
 * @author      Thomas
 * @version     V1.3
 * @date        11 Nov 2018
 * @brief       Source file for the Generic GenBuffer Class handle (template)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_GENBUF_HD

#ifndef GENBUFFER_CPP_              // As this class contains a template format, need to include
#define GENBUFFER_CPP_              // the source file within the header, therefore protection is
                                    // required from multiple loops.
template <typename Typ>
void GenBuffer<Typ>::Flush(void) {
/**************************************************************************************************
 * Function goes through the contents of the buffer, and writes everything to "0", and then
 * returns the input/output pointers back to the start of the buffer - ready for new data
 *************************************************************************************************/
    uint32_t i;                     // Variable used to loop through the entries within the buffer

    for (i = 0; i != length; i++) { // Start looping through buffer
        pa[i] = 0;                  // Write the data back to "0"
    }

    output_pointer  = 0;            // Initialise pointers back to the start of the buffer
    input_pointer   = 0;            // Initialise pointers back to the start of the buffer
}

template <typename Typ>
void GenBuffer<Typ>::QFlush(void) {
/**************************************************************************************************
 * Quick clear of the buffers, by setting all pointers to 0.
 *************************************************************************************************/
    output_pointer  = 0;            // Initialise pointers back to the start of the buffer
    input_pointer   = 0;            // Initialise pointers back to the start of the buffer
}

template <typename Typ>
GenBuffer<Typ>::GenBuffer(Typ *arrayloc, uint32_t size) {
/**************************************************************************************************
 * "Lite" function for GenBuffer.
 * Where the fully defined array is provided as input to constructor. This will then be linked to
 * internal class pointer "pa"
 *
 * Once done it will call the "QFlush" function to  setup pointers to the start of the buffer.
 *  Leaves the contents of the data unaffected.
 *************************************************************************************************/
    length = size;                  // Setup size of the buffer as per input
    pa = arrayloc;                  // Have pointer now point to input "arrayloc"

    QFlush();                       // Flush the data to default values
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef __LiteImplement__       // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
template <typename Typ>
GenBuffer<Typ>::GenBuffer() {
/**************************************************************************************************
 * Default constructor, that demands a buffer to be 32 entries deep. It will then generate a
 * new array structure and link to the private pointer "pa"
 *
 * Once done it will call the "Flush" function to write contents, and setup pointers to the start
 * of the buffer.
 *************************************************************************************************/
    length = 32;                    // Setup class variable "length" to 32
    pa = new Typ[length];           // Generate array

    Flush();                        // Flush the data to default values
}

template <typename Typ>
GenBuffer<Typ>::GenBuffer(uint32_t size) {
/**************************************************************************************************
 * Overloaded constructor, where the desired "size" of the buffer is provided to the class.
 * The rest of the setup is the same as the default constructor.
 *      It will generate a new array structure of the desired size, and link to the private poiner
 *      "pa"
 *
 * Once done it will call the "Flush" function to write contents, and setup pointers to the start
 * of the buffer.
 *************************************************************************************************/
    length = size;                  // Setup size of the buffer as per input
    pa = new Typ[length];           // Generate array

    Flush();                        // Flush the data to default values
}

template <typename Typ>
void GenBuffer<Typ>::SizeUpdate(uint32_t size) {
/**************************************************************************************************
 * Function to increase the size of the buffer.
 *************************************************************************************************/
    Typ *newpa;                     // Pointer to the new sized buffer
    newpa = new Typ[size];          // Generate new array

    uint32_t i;                     // Variable used to loop through the entries within the buffer
    for (i = 0; i != size; i++) {   // Loop through the new array
        if (i < length) {           // If at a point where the old buffer is still valid
            newpa[i] = pa[i];       // copy across the data
        }
        else
            newpa[i] = 0;           // Otherwise populate with "0"
    }

    length = size;                  // Copy across the new size of the buffer

    delete [] pa;                   // Delete the old array
    pa = newpa;                     // Update class array pointer


    input_pointer   = (input_pointer % length);     // Limit the pointers to the new size
    output_pointer  = (output_pointer % length);    // Limit the pointers to the new size
}

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

template <typename Typ>
_GenBufState GenBuffer<Typ>::State(void) {
/**************************************************************************************************
 * Function to determine the state of the Buffer - Empty/NewData/Full
 * It does this in 3 steps:
 *      1st     If the input pointer is equal to the output pointer. Then the buffer is empty
 *              and data can be added to the Buffer
 *
 *      2nd     If the output pointer is 1 in front of the input pointer, then the buffer is full
 *              (This does mean that the amount of data that can actually be written into the
 *              buffer is 1 minus the defined size)
 *
 *      3rd     If none of the above are true, then there is new data within the buffer which can
 *              be read
 *
 * Diagram:
 *                                     V Output pointer
 *             -------------------------------------------------------------------------------
 * Buffer ->  |    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |
 *             -------------------------------------------------------------------------------
 *                           ^3        ^1        ^2  <- Input pointer
 *  Size is 8 deep
 *
 *      When the input pointer is at position 1 (equal to the Output pointer), then the buffer
 *      is EMPTY, and new data can be added
 *
 *      When the input pointer is at position 2 (just infront of Output pointer + anywhere else)
 *      then there is NEWDATA which needs to be read by the Output
 *
 *      When the input pointer is at position 3 (just behind the Output pointer), then the buffer
 *      is FULL. As data within entry [2] has NOT been read yet.
 *************************************************************************************************/
    if      (output_pointer == input_pointer)                   // If the pointers are equal
        return (GenBuffer_Empty);                               // then buffer is empty

    else if (output_pointer == ((input_pointer + 1) % length))  // If output pointer is one behind
        return (GenBuffer_FULL);                                // the input, then buffer is full

    else                                                        // If none of the above are true
        return (GenBuffer_NewData);                             // then there is data in the buffer
                                                                // which needs to be read
}

template <typename Typ>
void GenBuffer<Typ>::InputWrite(Typ newdata) {
/**************************************************************************************************
 * Function will add data onto the buffer.
 * Then increase the input pointer, and limit it to the defined size of the buffer.
 * After this has been increased, if the buffer is considered "Empty" (input and output pointers
 * are equal) then need to increment the output pointer as well.
 * So as to maintain the oldest point of data within the buffer is limited to the size of the
 * buffer.
 *************************************************************************************************/
    pa[input_pointer] = newdata;                    // Add the input data into the buffer
    input_pointer = (input_pointer + 1) % length;   // Increment the input pointer, then take the
    // Modulus of this. This will then cause the input_pointer to be circled round if it equal to
    // the length.

    // If after the input pointer has been increased it is equal to the output pointer - therefore
    // is considered "Empty".
    // Then to force buffer size to be maintained as per "length", need to increase the output
    // pointer (essentially ensuring that the oldest point in the buffer is limited to "length"
    // old)
    if (State() == GenBuffer_Empty)                     // Check the buffer state, if equal to
                                                        // EMPTY, then
        output_pointer = (output_pointer + 1) % length; // Increase the output pointer by 1,
                                                        // limited to size "length"
}

template <typename Typ>
_GenBufState GenBuffer<Typ>::OutputRead(Typ *readdata)
/**************************************************************************************************
 * Function will take data from the buffer.
 * It will only provide an updated output if the buffer contains data (i.e. is not empty), if it
 * is empty then it will attempt nothing and return the empty indication
 *
 * If there is data, again it will be linked to the pointed data. Then it will increase the output
 * pointer, and limit it to the defined size of the buffer.
 *************************************************************************************************/
{
    _GenBufState returnentry = State();     // Generate variable to store the current state of
                                            // buffer, and update with latest state

    if (!(returnentry == GenBuffer_Empty)) {// If the buffer is not empty (therefore there is data
                                            // be read
        *readdata = pa[output_pointer];     // Update the output with the latest entry from buffer
        output_pointer = (output_pointer + 1) % length; // Increase the output pointer by 1,
                                                        // limited to size "length"
        return(returnentry);                // Return state of buffer prior to read
    }
    else                                    // If state is equal to "Empty"
        return (returnentry);               // Return state of buffer (which will be "Empty")
}

template <typename Typ>
Typ GenBuffer<Typ>::ReadBuffer(uint32_t position) {
/**************************************************************************************************
 * Simple function to just read a specific entry within the buffer, whilst limiting it to the size
 * of the buffer.
 *************************************************************************************************/
    position %= length;         // Limit and loop the position to ensure it is within the buffer

    return (pa[position]);      // Return buffer entry
}

template <typename Typ>
uint32_t GenBuffer<Typ>::SpaceRemaining(void) {
/**************************************************************************************************
 * Calculates the number of entries the buffer can take, before Buffer is FULL.
 *  Whereas the "InputWrite" will ensure that even if the buffer is full, it populates the latest
 *  data into the queue. This function will return "0" if the buffer is FULL, if the buffer is
 *  empty, then the output of this will be length - 1 (to cater for the buffer requiring 1 blank
 *  entry between input and output.
 *************************************************************************************************/
    if (State() == GenBuffer_Empty) {
        return (length - 1);
    }
    else { return((uint32_t)( output_pointer - input_pointer + length - 1 ) % length); }
        // Difference between the output and input pointers (note input is likely to be larger
        // than the output), take this difference and add to the length
}

template <typename Typ>
uint32_t GenBuffer<Typ>::SpaceTilArrayEnd(void) {
/**************************************************************************************************
 * Calculates the number of entries remaining within the source array, till the bottom is reached.
 *  At which point the GenBuffer will loop.
 *************************************************************************************************/
    return (length - input_pointer );
}

template <typename Typ>
uint32_t GenBuffer<Typ>::UnreadCount(void) {
/**************************************************************************************************
 * Calculates the number of entries within the buffer which have not been read yet.
 * If the buffer is already full, then it will return the full buffer size.
 *************************************************************************************************/
    if (State() == GenBuffer_FULL) {
        return (length - 1);
    }
    else { return((uint32_t)( length + input_pointer - output_pointer ) % length); }
}

template <typename Typ>
void GenBuffer<Typ>::QuickWrite(Typ *newdata, uint32_t size) {
/**************************************************************************************************
 * Populates the entries from "newdata" and puts into the GenBuffer. Only "size" entries are
 * copied.
 *  Relies upon the function "InputWrite"
 *************************************************************************************************/
    while(size != 0) {              // Cycle through the number of requested inputs
        InputWrite(*newdata);       // Put into array (also does loop back)
        newdata++;                  // Increase data pointer
        size--;                     // Decrease size (till = 0)
    }
}

template <typename Typ>
uint32_t GenBuffer<Typ>::QuickRead(Typ *backdata, uint32_t size) {
/**************************************************************************************************
 * Goes through the contents of the Buffer, and returns the data into the return array "backdata".
 * Will only cycle through "size" number of entries, actual entries returned is captured within
 * "retsize"
 *
 * Populates the entries from "backdata" and puts into the GenBuffer. Only "size" entries are
 * copied.
 *  Relies upon the function "InputWrite"
 *************************************************************************************************/
    uint32_t returnsize = 0;    // Initialise a count of the entries returned.
    Typ BufEntry = 0;           // Data variable to retain buffer entry

    while(size != 0) {          // Cycle through the number of data points to return
        if (OutputRead(&BufEntry) != GenBuffer_Empty) { // If buffer is not empty
            *backdata = BufEntry;                       // Return data to input array
            backdata++;                                 // Increase data pointer
            returnsize++;                               // Increment the return size count
            size--;                                     // Decrease size (till = 0)
        }
        else                                        // If buffer is empty
            break;                                  // exit loop
    }                                               // return size count will already be updated

    return (returnsize);                    // Return the number of entries populated
}

template <typename Typ>
void GenBuffer<Typ>::WriteErase(uint32_t size) {
/**************************************************************************************************
 * Bring the input_pointer forward by "size" number of entries - erasing "size" number of buffer
 * entries from being written too.
 *************************************************************************************************/
    if (SpaceRemaining() >= size) {
        input_pointer = (input_pointer + size) % length;
        // Bring the input pointer forward by specified amount. So long as the space remaining is
        // enough to allow this.
    }
    else
        input_pointer = (input_pointer + SpaceRemaining()) % length;
    // Otherwise, bring the input_pointer such that it is at the FULL threshold of the buffer
}

template <typename Typ>
void GenBuffer<Typ>::ReadErase(uint32_t size) {
/**************************************************************************************************
 * Bring the output_pointer forward by "size" number of entries - erasing "size" number of buffer
 * entries from being read from.
 *************************************************************************************************/
    if (UnreadCount() >= size) {
        output_pointer = (output_pointer + size) % length;
        // Bring the input pointer forward by specified amount. So long as the space remaining is
        // enough to allow this.
    }
    else
        output_pointer = input_pointer;
    // Otherwise, bring the input_pointer such that it is at the FULL threshold of the buffer
}

template <typename Typ>
GenBuffer<Typ>::~GenBuffer() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    delete [] pa;               // Delete the array "pa"

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

#endif
