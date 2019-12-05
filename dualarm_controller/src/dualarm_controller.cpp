//
// Created by june on 19. 12. 5..
//

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>
#include <Eigen/Dense>

#define _USE_MATH_DEFINES

namespace  dualarm_controller{
    class ComputedTorque_Control_CLIK : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {

        }
    };
};
PLUGINLIB_EXPORT_CLASS(dualarm_controller::ComputedTorque_Control_CLIK,
        controller_interface::ControllerBase)