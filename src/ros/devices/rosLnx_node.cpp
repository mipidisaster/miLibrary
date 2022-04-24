/**************************************************************************************************
 * @file        rosLnx_node.cpp
 * @author      Thomas
 * @brief       ROS node for the LnxCond class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "lnx_node"
 *   Configuration parameters:
 *      ~/config        /loop_rate
 *                      [ Floating data value, specifying the rate that the Linux device is
 *                      [ interrogated.
 *                      [ - Note, if none is provided will default to '4Hz'
 *
 *   Publishers:
 *      processorState
 *                      [ Provides the processor load of the embedded linux device, providing the
 *                      [ overal percentage load as well as individual cores, as well as the
 *                      [ temperature of the core.
 *                      [ Uses milibrary/msg/ProcessorUtilisation.msg
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      None
 *
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Project Libraries
// -----------------
#include "milibrary/ProcessorUtilisation.h"

#include "FileIndex.h"
// ~~~~~~~~~~~~~~~~~~~
#include FilInd_LnxCondHD               // Header for Linux Condition

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
class rosLnx {
private:
    // Constant(s) used within the class.
    std::string         kconfig_sub_area            = "config/";

    std::string         knode_loop_rate             = "loop_rate";

    std::string         kLnxCond_publish_cores      = "processorState";

private:
    LnxCond                 *_hardware_handle_  = NULL;

    std::vector<float>      _cpu_cores_;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    ros::NodeHandle         _nh_;
    ros::NodeHandle         _private_nh_;

    ros::NodeHandle         _nh_hardware_;
    ros::CallbackQueue      _hardware_callback_queue_;

    // PARAMETERS
    ////////////////////////
    double                          _loop_rate_parameter_;

    // MESSAGES
    ////////////////////////
    milibrary::ProcessorUtilisation _cpu_utilisation_message_;

    // PUBLISHERS
    ////////////////////////
    ros::Publisher                  _cpu_utilisation_publisher_;

    // SUBSCRIBERS
    ////////////////////////

    // TIMERS
    ////////////////////////

    // SERVICES
    ////////////////////////

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the LnxCond node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosLnx class
 */
    rosLnx(ros::NodeHandle* normal, ros::NodeHandle* private_namespace):
    _nh_(*normal), _private_nh_(*private_namespace)
    {
        //_nh_hardware_.setCallbackQueue(&_hardware_callback_queue_);

        if (configNode() < 0) {
            ROS_ERROR("Error detected during LnxCond construction, exiting node...");
            return;
        }

        nodeLoop();
    }

    /*
     *  @brief:  Separate function, to handle the hardware service callback queue.
     *           Intended to be used within a dedicated thread.
     *
     *  @param:  void
     *  @retval: void
     */
    //void hardwareCallbackThread(void) {
    //    ros::SingleThreadedSpinner spinner;
    //    spinner.spin(&_hardware_callback_queue_);
    //}

    /*
     *  @brief:  Function to encapsulate the looping of this node.
     *
     *  @param:  void
     *  @retval: void
     */
    void nodeLoop(void) {
        ROS_INFO("LnxCond node ready for use");

        ros::Rate loop_rate(_loop_rate_parameter_);

        _cpu_utilisation_message_.header.seq    = 0;
        _cpu_utilisation_message_.temperature   = -999;
        _cpu_utilisation_message_.fault         = (uint8_t) LnxCond::DevFlt::kInitialised;

        _cpu_cores_.insert( _cpu_cores_.begin(), LNX_NUM_CORES, -1.00f  );

        while (ros::ok()) {
            if (_hardware_handle_->updateStatus() != LnxCond::DevFlt::kNone) {
                _cpu_utilisation_message_.temperature   = -999;
                _cpu_utilisation_message_.overall       = -1;
                _cpu_utilisation_message_.cores         = _cpu_cores_;
            }
            else {
                _cpu_utilisation_message_.temperature   = _hardware_handle_->temperature;
                _cpu_utilisation_message_.overall       = (_hardware_handle_->cpu_load[0] * 100);

                for (int i = 0; i != _cpu_cores_.size(); i ++) {
                    _cpu_cores_[i] = (_hardware_handle_->cpu_load[i + 1] * 100);
                }
                _cpu_utilisation_message_.cores     = _cpu_cores_;

            }

            _cpu_utilisation_message_.fault             = (uint8_t) _hardware_handle_->flt;
            _cpu_utilisation_message_.header.stamp      = ros::Time::now();

            _cpu_utilisation_publisher_.publish(_cpu_utilisation_message_);


            ros::spinOnce();
            loop_rate.sleep();
            _cpu_utilisation_message_.header.seq++;
        }
    }

    /*
     *  @brief:  Setups the LnxCond for the node, as per the expected input/configuration
     *           parameters from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        // Input parameters
        _private_nh_.param<double>(kconfig_sub_area + knode_loop_rate,
                                   _loop_rate_parameter_,
                                   4.0);

        //=========================================================================================
        // Duplication check
        // Not needed for this node

        //=========================================================================================

        _hardware_handle_ = new LnxCond();

        ROS_INFO("LnxCond has been setup");
        //=========================================================================================
        // Publishers/Subscribers
        _cpu_utilisation_publisher_   = _nh_.advertise<milibrary::ProcessorUtilisation>(
                                                kLnxCond_publish_cores,
                                                5);

        //=========================================================================================
        // Timers
        // None

        //=========================================================================================
        // Clients/Servers

        //=========================================================================================

        ROS_INFO("LnxCond node constructed");
        return 0;   // If got to this point, no errors were detected
    }

    ~rosLnx() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linux_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosLnx  node(&n, &private_params);

    ros::shutdown();            // If get to this point of the main() function. Then should be
                                // shutdowning the node - due to error
    ros::waitForShutdown();

    // On node shutdown, don't think it reaches this part of main()
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

