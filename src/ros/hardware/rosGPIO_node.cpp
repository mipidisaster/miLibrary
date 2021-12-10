/**************************************************************************************************
 * @file        rosGPIO_node.cpp
 * @author      Thomas
 * @brief       ROS node for the GPIO class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "gpio_node"
 *   Configuration parameters:
 *      ~/config        /output/gpio_pin
 *                      /input/gpio_pin
 *                      [ Both of these parameters are expected to be lists of gpios pin numbers,
 *                      [ left most entry is the LSB
 *
 *   Publishers:
 *      None
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      set_gpio        [ Request-> Value for the GPIO output to be set too
 *                      [ Result -> Nothing
 *                      [ Uses GPIOctrl.srv
 *
 *************************************************************************************************/
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
// ---------------
#include "ros/ros.h"

// Project Libraries
// -----------------
#include "milibrary/GPIOctrl.h"

#include "FileIndex.h"
// ~~~~~~~~~~~~~~~~~~~
#include FilIndROSNode_TP               // Template for the miROSnode file
#include FilInd_GPIO___HD               // Include the GPIO class handler

//=================================================================================================
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
class rosGPIO: public miROSnode {
private:
    // Constant(s) used within the class.
    std::string         kinput_gpio_param           = "input/";
    std::string         koutput_gpio_param          = "output/";

    std::string         kGPIO_set_service           = "set_gpio";

private:
    std::vector<GPIO>   _output_array_;
    std::vector<GPIO>   _input_array_;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    // PARAMETERS
    ////////////////////////
    // _input_gpio_params_ and _output_gpio_params_ from miROSnode

    // MESSAGES
    ////////////////////////

    // PUBLISHERS
    ////////////////////////

    // SUBSCRIBERS
    ////////////////////////

    // TIMERS
    ////////////////////////

    // SERVICES
    ////////////////////////
    ros::ServiceServer      _set_server_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the GPIO node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosGPIO class
 */
    rosGPIO(ros::NodeHandle* normal, ros::NodeHandle* private_params):
    miROSnode(normal, private_params)
    {

    }

    /*
     *  @brief:  Setups the GPIOs for the node, as per the expected input/configuration parameters
     *           from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *
     *  @param:  void
     *  @retval: integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        ParamStatus output_parameters_good = rosParamCheck(
                                kconfig_sub_area + koutput_gpio_param + kgpio_generic_param,
                                _output_gpio_params_);

        if          (output_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("Output ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      (kconfig_sub_area + koutput_gpio_param + kgpio_generic_param).c_str());
        } else if   (output_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("Output ROS Parameter %s was not within roscore",
                    (kconfig_sub_area + koutput_gpio_param + kgpio_generic_param).c_str());
        } else {
            ROS_INFO("The following gpios have been provided as output pins:");

            for (int i = 0; i != _output_gpio_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _output_gpio_params_.at(i));
            }
        }
        //=========================================================================================
        ParamStatus input_parameters_good  = rosParamCheck(
                                kconfig_sub_area + kinput_gpio_param + kgpio_generic_param,
                                _input_gpio_params_);

        if          (input_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("Input ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      (kconfig_sub_area + kinput_gpio_param + kgpio_generic_param).c_str());
        } else if   (input_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("Input ROS Parameter %s was not within roscore",
                    (kconfig_sub_area + kinput_gpio_param + kgpio_generic_param).c_str());
        } else {
            ROS_INFO("The following gpios have been provided as input pins:");

            for (int i = 0; i != _input_gpio_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _input_gpio_params_.at(i));
            }
        }

        if          ( (output_parameters_good == ParamStatus::kParameter_present_but_invalid)
                ||    (input_parameters_good  == ParamStatus::kParameter_present_but_invalid) ) {
            // If there was one of the ROS parameters provided, but of the wrong type. Assume that
            // this 'should' be set - so need to get user to correct error. Return error
            ROS_ERROR("One of the inputs provided is of the wrong type, please either "
                      "correct or remove the parameter from the namespace: to allow node to "
                      " be generated");
            return -1;
        } else if   ( (output_parameters_good != ParamStatus::kParameter_present)
                &&    (input_parameters_good  != ParamStatus::kParameter_present) ) {
            ROS_ERROR("Expected ROS Parameters not within roscore, or not valid");
            return -1;
        }

        //=========================================================================================
        ROS_INFO("Checking that there is no overlap of gpio numbers...");
        if(duplicationParamCheck() < 0) {       // If duplication(s) detected
            return -1;                          // exit
        }
        ROS_INFO("No GPIO duplications detected");

        //=========================================================================================
        ROS_INFO("Generating the GPIOs");
        // Input parameters how now been checked, and any errors would of exitted before getting
        // here...
        GPIO::piSetup();        // Needed to make GPIO pins work

        // 'at' version of the std::vector used instead of the operator[], as this provides
        // detection of accessing something outside the bounds of the pinaddress.
        // Generate the array of 'output' GPIOs
        for (int i = 0; i != _output_gpio_params_.size(); i++) {
            _output_array_.push_back(   GPIO(   GPIO::State::kLow,
                                                _output_gpio_params_.at(i),
                                                GPIO::Dir::kOutput)  );

            ROS_INFO("Output --> GPIO[%i]\tconstructed as bit %i", _output_gpio_params_.at(i), i);
        }

        for (int i = 0; i != _input_gpio_params_.size(); i++) {
            _input_array_.push_back(    GPIO(   GPIO::State::kLow,
                                                _input_gpio_params_.at(i),
                                                GPIO::Dir::kInput)  );

            ROS_INFO("Input --> GPIO[%i]\tconstructed as bit %i", _input_gpio_params_.at(i), i);
        }

        ROS_INFO("GPIO has been setup");
        //=========================================================================================
        _set_server_        = _nh_.advertiseService(kGPIO_set_service,
                                                    &rosGPIO::callbackGPIOset,
                                                    this);

        ROS_INFO("GPIO Server constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Basic function, will take the input value, and configure the GPIOs within node
     *           to display that value - in binary
     *           Essentially a wrapper for the GPIO function - 'setGPIOArray'
     *
     *  @param:  unsigned 32bit value
     *  @retval: void
     */
    void setGPIOvalue(uint32_t new_value)
    {
        GPIO::setGPIOArray(_output_array_.data(), _output_array_.size(), new_value);
    }

    /*
     *  @brief:  Callback function to be used as part of the service of this node. Will receive
     *           in the GPIOctrl message, to set the GPIOs constructed in this class to the desired
     *           value
     *
     *  @param:  milibrary GPIOCtrl request
     *  @param:  milibrary GPIOCtrl response (nothing)
     *  @retval: Service needs to return a boolean type
     */
    bool callbackGPIOset(milibrary::GPIOctrl::Request &req, milibrary::GPIOctrl::Response &res) {
        setGPIOvalue(req.value);
        return true;
    }

    ~rosGPIO() {
        ROS_INFO("Shuting down node, and returning pins to FALSE");
        // Drive the GPIOs to a default value of "0"
        setGPIOvalue( 0 );
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosGPIO  node_GPIO(&n, &private_params);
    if (node_GPIO.configNode() < 0) {
        ROS_ERROR("Error detected during GPIO construction, exiting node...");
        return -1;
    }

    ROS_INFO("GPIO node ready for use");
    ros::spin();

    // On node shutdown, don't think it reaches this part of main()
    // However, will call class destroyer
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None

