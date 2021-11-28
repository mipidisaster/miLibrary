/**************************************************************************************************
 * @file        rosGPIO_node.cpp
 * @author      Thomas
 * @brief       ROS node for the GPIO class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

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
    std::string         kinput_gpio_param   = "input_gpios";
    std::string         koutput_gpio_param  = "output_gpios";

    std::string         kGPIO_value_service = "update_gpio";


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
    std::vector<int>        _input_params_;
    std::vector<int>        _output_params_;

    // PUBLISHERS
    ////////////////////////

    // SUBSCRIBERS
    ////////////////////////

    // SERVICES
    ////////////////////////
    ros::ServiceServer      _gpio_setting_server_;

    // Actions
    ////////////////////////

public:
/*
 *  @brief:  Create the GPIO node class, which will handle the checking of the input 'rosparams' to
 *           ensure that they are valid for GPIO use.
 *           Will also check the other spawns of this node/class to ensure that there is NO
 *           duplication of the same GPIO pin.
 *           //TODO: Will need to put in some kind of check to ensure that any other calls/use of
 *                   rosparam 'gpio_address' do not overlap - possible with some kind of check
 *                   against SPI/USART/etc. calls
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosGPIO class
 */
    rosGPIO(ros::NodeHandle* normal, ros::NodeHandle* private_params):
    miROSnode(normal, private_params)
    {
        _gpio_setting_server_ = _nh_.advertiseService(kGPIO_value_service,
                                                      &rosGPIO::callbackGPIOset,
                                                      this);
    }

    /*
     *  @brief:  Will retrieve any and all instances of the input/output GPIO parameters, and
     *           ensure that there is no replication of the gpios for this node.
     *           Will display a message for each duplication detected
     *  @retval: Integer --
     *                            "0"   = Input parameter(s) is good
     *                            "-1"  = Duplication(s) detected
     */
    int duplicationParamCheck(void)
    {
        std::vector<int> all_output_params  = allMatchingParam<int>(koutput_gpio_param);
        std::vector<int> all_input_params   = allMatchingParam<int>(kinput_gpio_param);
        std::vector<int> all_gpio_params(all_output_params);
        all_gpio_params.insert(all_gpio_params.begin(),
                               all_input_params.begin(), all_input_params.end());

        std::vector<std::string> keys;
        _nh_.getParamNames(keys);

        int return_value = 0;

        for (int i = 0; i != _output_params_.size(); i++) {
            int number_parameters = std::count(all_gpio_params.begin(), all_gpio_params.end(),
                                                _output_params_.at(i));

            if (number_parameters > 1) {
                ROS_ERROR("Duplication detected of the following output GPIO[%i]",
                           _output_params_.at(i));

                return_value = -1;
            }
        }

        for (int i = 0; i != _input_params_.size(); i++) {
            int number_parameters = std::count(all_gpio_params.begin(), all_gpio_params.end(),
                                                _input_params_.at(i));

            if (number_parameters > 1) {
                ROS_ERROR("Duplication detected of the following input GPIO[%i]",
                           _input_params_.at(i));

                return_value = -1;
            }
        }

        return return_value;
    }

    /*
     *  @brief:  Setups the GPIOs for the node, as per the expected input/configuration parameters
     *           from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *  @param:  void
     *  @retval: integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        ParamStatus output_parameters_good = rosParamCheck(koutput_gpio_param, _output_params_);

        if          (output_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("output ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      koutput_gpio_param.c_str());
        } else if   (output_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("output ROS Parameter %s was not within roscore",
                     koutput_gpio_param.c_str());
        } else {
            ROS_INFO("The following gpios have been provided as output pins:");

            for (int i = 0; i != _output_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _output_params_.at(i));
            }
        }
        //=========================================================================================
        ParamStatus input_parameters_good  = rosParamCheck(kinput_gpio_param, _input_params_);

        if          (input_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("input ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      kinput_gpio_param.c_str());
        } else if   (input_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("input ROS Parameter %s was not within roscore",
                      kinput_gpio_param.c_str());
        } else {
            ROS_INFO("The following gpios have been provided as input pins:");

            for (int i = 0; i != _input_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _input_params_.at(i));
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

        //=========================================================================================
        ROS_INFO("Generating the GPIOs");
        // Input parameters how now been checked, and any errors would of exitted before getting
        // here...
        GPIO::piSetup();        // Needed to make GPIO pins work

        // 'at' version of the std::vector used instead of the operator[], as this provides
        // detection of accessing something outside the bounds of the pinaddress.
        // Generate the array of 'output' GPIOs
        for (int i = 0; i != _output_params_.size(); i++) {
            _output_array_.push_back(   GPIO(   GPIO::State::kLow,
                                                _output_params_.at(i),
                                                GPIO::Dir::kOutput)  );

            ROS_INFO("output --> GPIO[%i] constructed as bit %i", _output_params_.at(i), i);
        }

        for (int i = 0; i != _input_params_.size(); i++) {
            _input_array_.push_back(    GPIO(   GPIO::State::kLow,
                                                _input_params_.at(i),
                                                GPIO::Dir::kInput)  );

            ROS_INFO("input --> GPIO[%i] constructed as bit %i", _input_params_.at(i), i);
        }

        ROS_INFO("GPIO has been setup");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Basic function, will take the input value, and configure the GPIOs within node
     *           to display that value - in binary
     *           Essentially a wrapper for the GPIO function - 'setGPIOArray'
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
     *  @param:  milibrary GPIOCtrl request
     *  @param:  milibrary GPIOCtrl response (nothing)
     *  @retval: Service needs to return a boolean type
     */
    bool callbackGPIOset(milibrary::GPIOctrl::Request &req, milibrary::GPIOctrl::Response &res) {
        setGPIOvalue(req.value);
        return true;
    }


    ~rosGPIO() {
        // !!!DESTROY THINGS!!!
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

    // Set output to a value of "0" ready for next use
    ROS_INFO("Shuting down node, and returning pins to FALSE");
    node_GPIO.setGPIOvalue( 0 );

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

