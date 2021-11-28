/**************************************************************************************************
 * @file        miROSnode.cpp
 * @author      Thomas
 * @brief       Header file for the ROS node class template
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class will include generic parameters and functions which will be used by my ROS hardware
 * nodes.
 * Use of class:
 *      When constructed need to provide the NodeHandles generated from main(). One is to general
 *      version, the second is to be the private NodeHandle version (called with ("~")).
 *
 *      Will provide the following:
 *          Enumate type ("ParamStatus") for status of array/list type ros parameters (seems to
 *          work with "integers" so far)
 *
 *          Template functions:
 *          ".rosParamCheck"        - Will check for parameter within the private NodeHandle,
 *                                    expects the parameter to be a list/array.
 *          ".allMatchingParam"     - Will retrieve all instances of the same parameter name from
 *                                    all of namespaces.
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef MIROSNODE_TEMPLATE_
#define MIROSNODE_TEMPLATE_

// C System Header(s)
// ------------------
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings
#include <algorithm>    // for std::count

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz ==  50)      // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#error "Not supported"

#elif (zz__MiEmbedType__zz ==  51)      // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#error "Not supported"

#elif (zz__MiEmbedType__zz ==  10)      // If the target device is an Raspberry Pi then
//=================================================================================================
#include "ros/ros.h"

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
#include "ros/ros.h"

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
// None

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class miROSnode {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "miROSnode::" followed by the type.
 *************************************************************************************************/
public:
    enum class ParamStatus : uint8_t {
        kParameter_present                  = 0x00,
        kParameter_not_present              = 0x01,

        kParameter_present_but_invalid      = 0x02
    };

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
protected:
    ros::NodeHandle     _nh_;
    ros::NodeHandle     _private_params_;

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
public:
    miROSnode(ros::NodeHandle* normal, ros::NodeHandle* private_params);

    virtual ~miROSnode();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "miROSnode" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>  GENERIC FUNCTIONS FOR HARDWARE ROS NODES <<<
             *   -----------
             *  Functions which are to be used by my hardware ROS nodes, so as to generalise
             *  functions between these nodes (i.e. not constantly re-generating code)
             *************************************************************************************/
    template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
    ParamStatus rosParamCheck(const std::string& key, std::vector<_Tp, _Alloc>& vec);

    template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
    std::vector<_Tp, _Alloc> allMatchingParam(const std::string& search_parameter);
};

miROSnode::miROSnode(ros::NodeHandle* normal, ros::NodeHandle* private_params):
/**************************************************************************************************
 * Construct the miROSnode, with the pointers to the NodeHandles for the generic and private
 * namespaces.
 * Nothing else is really constructed within this class
 *************************************************************************************************/
_nh_(*normal), _private_params_(*private_params) {

}

template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
miROSnode::ParamStatus miROSnode::rosParamCheck(const std::string& key,
                                                   std::vector<_Tp, _Alloc>& vec) {
/**************************************************************************************************
 * Similar construction as per the ros -'getParam', however this version will do a check to see if
 * the parameter exists, and that the parameter contains at least one entry (expecting a
 * array/list of some kind).
 *************************************************************************************************/
    if ( _private_params_.hasParam(key) ) {
        _private_params_.getParam(key, vec);

        if ( vec.size() < 1 )
                return ParamStatus::kParameter_present_but_invalid;
    }
    else
        return ParamStatus::kParameter_not_present;

    return ParamStatus::kParameter_present;
}

/*
 *  @brief:  Will scan through all the ros parameters in the roscore space, and look for a
 *           match with the input key
 *  @param:  const standard string, with the rosparam name
 *  @retval: Vector structure of integers, for the input key (not will only capture integer(s))
 */
template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
std::vector<_Tp, _Alloc> miROSnode::allMatchingParam(const std::string& search_parameter) {
/**************************************************************************************************
 * Will scan through all the ros parameters in the roscore space, and look for a match with the
 * input key.
 * Provides as output a complete of all the entries which match this key (type dependent)
 *************************************************************************************************/
    std::vector<std::string> keys;
    _nh_.getParamNames(keys);

    std::vector<_Tp, _Alloc> all_params;
    std::vector<_Tp, _Alloc> temp;

    for (int i = 0; i != keys.size(); i++) {

        std::string temp_name = keys.at(i);

        if (  keys.at(i).length() < search_parameter.length()  )
            continue;

        if (  keys.at(i).compare(
                        keys.at(i).length() - search_parameter.length(),
            // Start position of where the ros parameter name should be located (at the end)
                        search_parameter.length(),
            // Length of ros parameter
                        search_parameter) == 0  )
        {
            _nh_.getParam(keys.at(i).c_str(), temp);
            all_params.insert(all_params.begin(), temp.begin(), temp.end());

            temp.clear();
        }
    }

    return all_params;
}

miROSnode::~miROSnode() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    // Nothing to do

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// Nothing to do

#else
//=================================================================================================
// Nothing to do

#endif
}

#endif /* MIROSNODE_HPP_ */
