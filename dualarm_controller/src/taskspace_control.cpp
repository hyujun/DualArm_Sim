//
// Created by june on 19. 12. 5..
//

// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include "utils.h"
#include "dualarm_controller/TaskCurrentState.h"

//manipulability
#include <ellipsoid.h>
#include <similarity.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>                // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>          // jacobian
#include <kdl/chainjnttojacdotsolver.hpp>       // jcobian dot
#include <kdl/chainfksolverpos_recursive.hpp>   // forward kinematics

#include <boost/scoped_ptr.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include <SerialManipulator.h>
#include <Controller.h>

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

#define A 0.10
#define b1 0.45
#define b2 -0.30
#define b3 0.45
#define f 0.2

#define l_p1 0.40
#define l_p2 0.30
#define l_p3 0.39

#define Deg_A 70
#define Deg_f 0.5

namespace dualarm_controller
{
    class Taskspace_Control : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // ********* 1. Get joint name / gain from the parameter server *********
            // 1.0 Target Position objective
            if (!n.getParam("target_obj", target_obj))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            if (!n.getParam("ctrl_type", ctrl_type))
            {
                if(ctrl_type != 1 && ctrl_type != 2)
                {
                    ROS_ERROR("Could not find controller mode");
                    return false;
                }
            }

            // 1.1 Joint Name
            if (!n.getParam("joints", joint_names_))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }
            n_joints_ = joint_names_.size();

            if (n_joints_ == 0)
            {
                ROS_ERROR("List of joint names is empty.");
                return false;
            }
            else
            {
                ROS_INFO("Found %d joint names", n_joints_);
                for (int i = 0; i < n_joints_; i++)
                {
                    ROS_INFO("%s", joint_names_[i].c_str());
                }
            }

            // 1.2 Gain
            // 1.2.1 Task-space Controller

            if (!n.getParam("/dualarm/taskspace_control/gains/Regulation/pid/KpT", Kp_trans_R))
            {
                ROS_ERROR("Cannot find Right-arm pid/Kp Translation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Regulation/pid/KpR", Kp_rot_R))
            {
                ROS_ERROR("Cannot find Right-arm pid/Kp Rotation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Regulation/pid/KdT", Kd_trans_R))
            {
                ROS_ERROR("Cannot find Right-arm pid/Kd Translation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Regulation/pid/KdR", Kd_rot_R))
            {
                ROS_ERROR("Cannot find Right-arm pid/Kd Rotation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Tracking/pid/KpT", Kp_trans_L))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kp Translation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Tracking/pid/KpR", Kp_rot_L))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kp Rotation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Tracking/pid/KdT", Kd_trans_L))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kd Translation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control/gains/Tracking/pid/KdR", Kd_rot_L))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kd Rotation gain");
                return false;
            }

            // 2. ********* urdf *********
            urdf::Model urdf;
            if (!urdf.initParam("robot_description"))
            {
                ROS_ERROR("Failed to parse urdf file");
                return false;
            }
            else
            {
                ROS_INFO_STREAM("Found robot_description");
            }

            // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
            for (int i = 0; i < n_joints_; i++)
            {
                try
                {
                    joints_.push_back(hw->getHandle(joint_names_[i]));
                }
                catch (const hardware_interface::HardwareInterfaceException &e)
                {
                    ROS_ERROR_STREAM("Exception thrown: " << e.what());
                    return false;
                }

                urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
                if (!joint_urdf)
                {
                    ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                    return false;
                }
                joint_urdfs_.push_back(joint_urdf);
            }

            // 4. ********* KDL *********
            // 4.1 kdl parser
            if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
            {
                ROS_ERROR("Failed to construct kdl tree");
                return false;
            }

            // 4.2 kdl chain
            std::string root_name, tip_name1, tip_name2;
            if (!n.getParam("root_link", root_name))
            {
                ROS_ERROR("Could not find root link name");
                return false;
            }
            if (!n.getParam("tip_link1", tip_name1))
            {
                ROS_ERROR("Could not find tip link name");
                return false;
            }
            if (!n.getParam("tip_link2", tip_name2))
            {
                ROS_ERROR("Could not find tip link name");
                return false;
            }

            if (!kdl_tree_.getChain(root_name, tip_name1, kdl_chain_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name1);
                ROS_ERROR_STREAM("  Chain has " << kdl_chain_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Chain has " << kdl_chain_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO_STREAM("Got kdl first chain");
                ROS_INFO_STREAM("  " << root_name << " --> " << tip_name1);
                ROS_INFO_STREAM("  Chain has " << kdl_chain_.getNrOfJoints() << " joints");
                ROS_INFO_STREAM("  Chain has " << kdl_chain_.getNrOfSegments() << " segments");
            }

            if(!kdl_tree_.getChain(root_name, tip_name2, kdl_chain2_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name2);
                ROS_ERROR_STREAM("  Chain has " << kdl_chain2_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Chain has " << kdl_chain2_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO_STREAM("Got kdl second chain");
                ROS_INFO_STREAM("  " << root_name << " --> " << tip_name2);
                ROS_INFO_STREAM("  Chain has " << kdl_chain2_.getNrOfJoints() << " joints");
                ROS_INFO_STREAM("  Chain has " << kdl_chain2_.getNrOfSegments() << " segments");
            }

            // 4.3 inverse dynamics solver 초기화
            g_kdl_ = KDL::Vector::Zero();
            g_kdl_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

            id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, g_kdl_));
            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            jnt_to_jacdot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            J1_kdl_.resize(kdl_chain_.getNrOfJoints());
            J1dot.resize(kdl_chain_.getNrOfJoints());
            M_kdl_.resize(kdl_chain_.getNrOfJoints());
            C_kdl_.resize(kdl_chain_.getNrOfJoints());
            G_kdl_.resize(kdl_chain_.getNrOfJoints());

            id_solver1_.reset(new KDL::ChainDynParam(kdl_chain2_, g_kdl_));
            jnt_to_jac_solver1_.reset(new KDL::ChainJntToJacSolver(kdl_chain2_));
            jnt_to_jacdot_solver1.reset(new KDL::ChainJntToJacDotSolver(kdl_chain2_));
            fk_pos_solver1_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain2_));
            J2_kdl_.resize(kdl_chain2_.getNrOfJoints());
            J2dot.resize(kdl_chain2_.getNrOfJoints());
            M1_kdl_.resize(kdl_chain2_.getNrOfJoints());
            C1_kdl_.resize(kdl_chain2_.getNrOfJoints());
            G1_kdl_.resize(kdl_chain2_.getNrOfJoints());

            Jdot.setZero(12,16);

            q1_.resize(9);
            q1dot_.resize(9);
            q2_.resize(9);
            q2dot_.resize(9);

            // ********* 5. 각종 변수 초기화 *********
            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(12);
            ex_.setZero(12);
            ex_dot_.setZero(12);
            dx.setZero(12);
            dxdot.setZero(12);
            dxddot.setZero(12);

            KpTask.setZero(12);
            KdTask.setZero(12);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            torque.setZero(n_joints_);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            state_pub_.reset(new realtime_tools::RealtimePublisher<dualarm_controller::TaskCurrentState>(n, "states", 10));
            state_pub_->msg_.header.stamp = ros::Time::now();
            //state_pub_->msg_.header.frame_id = "dualarm";
            state_pub_->msg_.header.seq = 0;
            for(int i=0; i<n_joints_; i++)
            {
                state_pub_->msg_.q.push_back(q_.data(i));
                state_pub_->msg_.qdot.push_back(qdot_.data(i));
                state_pub_->msg_.dq.push_back(qd_.data(i));
                state_pub_->msg_.dqdot.push_back(qd_dot_.data(i));
                state_pub_->msg_.torque.push_back(torque(i));
            }
            for(int j=0; j<2; j++) {
                state_pub_->msg_.InverseConditionNum.push_back(InverseConditionNumber[j]);
            }
            state_pub_->msg_.x.resize(2);
            state_pub_->msg_.dx.resize(2);
            state_pub_->msg_.DAMM = DAMM;
            state_pub_->msg_.TODAMM = TODAMM2;

            state_pub_->msg_.Kp_R = Kp_rot_R;
            state_pub_->msg_.Kp_T = Kp_trans_R;
            pub_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            // 6.2 subsriber
            const auto joint_state_cb = utils::makeCallback<dualarm_controller::TaskCurrentState>([&](const auto& msg){

            });
            sub_x_cmd_ = n.subscribe<dualarm_controller::TaskCurrentState>( "command", 5, joint_state_cb);

            return true;
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            InitTime=2.0;

            ROS_INFO("Starting Task-space Controller");

            cManipulator = std::make_shared<SerialManipulator>();

            Control = std::make_unique<HYUControl::Controller>(cManipulator);

            cManipulator->UpdateManipulatorParam();

            if(ctrl_type == 1)
            {
                KpTask.segment(0,3).setConstant(Kp_rot_R);
                KpTask.segment(3,3).setConstant(Kp_trans_R);
                KpTask.segment(6,3).setConstant(Kp_rot_R);
                KpTask.segment(9,3).setConstant(Kp_trans_R);

                KdTask.segment(0,3).setConstant(Kd_rot_R);
                KdTask.segment(3,3).setConstant(Kd_trans_R);
                KdTask.segment(6,3).setConstant(Kd_rot_R);
                KdTask.segment(9,3).setConstant(Kd_trans_R);
            }
            else if( ctrl_type == 2)
            {
                KpTask.segment(0,3).setConstant(Kp_rot_L);
                KpTask.segment(3,3).setConstant(Kp_trans_L);
                KpTask.segment(6,3).setConstant(Kp_rot_L);
                KpTask.segment(9,3).setConstant(Kp_trans_L);

                KdTask.segment(0,3).setConstant(Kd_rot_L);
                KdTask.segment(3,3).setConstant(Kd_trans_L);
                KdTask.segment(6,3).setConstant(Kd_rot_L);
                KdTask.segment(9,3).setConstant(Kd_trans_L);
            }

            Control->SetTaskspaceGain(KpTask, KdTask);
        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            std::vector<double> &commands = *pub_buffer_.readFromRT();
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();

            clock_gettime(CLOCK_MONOTONIC, &begin);

            // 0.2 joint state
            for (int i = 0; i < n_joints_; i++)
            {
                q_(i) = joints_[i].getPosition();
                qdot_(i) = joints_[i].getVelocity();
                //torque_(i) = joints_[i].getEffort();
            }

            //----------------------
            // dynamics calculation
            //----------------------
            cManipulator->pKin->PrepareJacobian(q_.data);
            cManipulator->pDyn->PrepareDynamics(q_.data, qdot_.data);

            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);
            cManipulator->pKin->GetAngleAxis(ForwardAxis, ForwardAngle, NumChain);
            cManipulator->pKin->GetInverseConditionNumber(InverseConditionNumber);

            q1_.data = q_.data.head(9);
            q1dot_.q = q1_;
            q1dot_.qdot.data = qdot_.data.head(9);

            q2_.data.tail(7) = q_.data.tail(7);
            q2_.data.head(2) = q_.data.head(2);
            q2dot_.q = q2_;
            q2dot_.qdot.data.tail(7) = qdot_.data.tail(7);
            q2dot_.qdot.data.head(2) = qdot_.data.head(2);

            fk_pos_solver_->JntToCart(q1_, x_[0]);
            fk_pos_solver1_->JntToCart(q2_, x_[1]);

            jnt_to_jac_solver_->JntToJac(q1_, J1_kdl_);
            jnt_to_jac_solver1_->JntToJac(q2_, J2_kdl_);

            jnt_to_jacdot_solver->JntToJacDot(q1dot_, J1dot);
            jnt_to_jacdot_solver1->JntToJacDot(q2dot_, J2dot);

            MM = cManipulator->pKin->GetManipulabilityMeasure();
            manipulability_data();

            if( t <= InitTime )
            {
                qd_.data(0) = -0.0*D2R;
                qd_.data(1) = -0.0*D2R;

                qd_.data(2) = 0.0*D2R;
                qd_.data(3) = -0.0*D2R;
                qd_.data(4) = -0.0*D2R;
                qd_.data(5) = -0.0*D2R;
                qd_.data(6) = -70.0*D2R;
                qd_.data(7) = 0.0*D2R;
                qd_.data(8) = 0.0*D2R;

                qd_.data(9) = 0.0*D2R;
                qd_.data(10) = 0.0*D2R;
                qd_.data(11) = 0.0*D2R;
                qd_.data(12) = -0.0*D2R;
                qd_.data(13) = 70.0*D2R;
                qd_.data(14) = -0.0*D2R;
                qd_.data(15) = -0.0*D2R;
            }
            else
            {
                if(target_obj == 0)
                {
                    xd_[0].p(0) = A * sin(f * M_PI * (t - InitTime)) + b1;
                    xd_[0].p(1) = b2;
                    xd_[0].p(2) = b3;
                    xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(0, 0, M_PI/2));

                    xd_[1].p(0) = -A * sin(f * M_PI * (t - InitTime)) + l_p1;
                    xd_[1].p(1) = l_p2;
                    xd_[1].p(2) = l_p3;
                    xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(-M_PI/2, 0, M_PI/2));

                    dxdot.setZero();
                    dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(9) = -(f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    x_[0].p(0) = ForwardPos[0](0);
                    x_[0].p(1) = ForwardPos[0](1);
                    x_[0].p(2) = ForwardPos[0](2);
                    x_[0].M = KDL::Rotation(KDL::Rotation::RPY(ForwardOri[0](0), ForwardOri[0](1), ForwardOri[0](2)));

                    x_[1].p(0) = ForwardPos[1](0);
                    x_[1].p(1) = ForwardPos[1](1);
                    x_[1].p(2) = ForwardPos[1](2);
                    x_[1].M = KDL::Rotation(KDL::Rotation::RPY(ForwardOri[1](0), ForwardOri[1](1), ForwardOri[1](2)));

                    ex_temp_ = diff(x_[0], xd_[0]);
                    ex_(0) = ex_temp_(3);
                    ex_(1) = ex_temp_(4);
                    ex_(2) = ex_temp_(5);
                    ex_(3) = ex_temp_(0);
                    ex_(4) = ex_temp_(1);
                    ex_(5) = ex_temp_(2);

                    ex_temp_ = diff(x_[1], xd_[1]);
                    ex_(6) = ex_temp_(3);
                    ex_(7) = ex_temp_(4);
                    ex_(8) = ex_temp_(5);
                    ex_(9) = ex_temp_(0);
                    ex_(10) = ex_temp_(1);
                    ex_(11) = ex_temp_(2);
                }
                if (target_obj == 1)
                {
                    dx(0) = 0;
                    dx(1) = -M_PI_2;
                    dx(2) = 0;
                    dx(3) = A * sin(f * M_PI * (t - InitTime)) + b1;
                    dx(4) = b2;
                    dx(5) = b3;

                    dx(6) = 0;
                    dx(7) = -M_PI_2;
                    dx(8) = 0;
                    dx(9) = -A * sin(f * M_PI * (t - InitTime)) + l_p1;
                    dx(10) = l_p2;
                    dx(11) = l_p3;

                    dxdot.setZero();
                    dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(9) = -(f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    dxddot.setZero();
                    dxddot(3) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                    dxddot(9) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                }
                else if (target_obj == 2)
                {
                    dx(0) = 0;
                    dx(1) = -M_PI_2;
                    dx(2) = 0;
                    dx(3) = b1;
                    dx(4) = A * sin(f * M_PI * (t - InitTime)) + b2;
                    dx(5) = b3;

                    dx(6) = 0;
                    dx(7) = -M_PI_2;
                    dx(8) = 0;
                    dx(9) = l_p1;
                    dx(10) = -A * sin(f * M_PI * (t - InitTime)) + l_p2;
                    dx(11) = l_p3;

                    dxdot.setZero();
                    dxdot(4) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(10) = -(f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    dxddot.setZero();
                    dxddot(4) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                    dxddot(10) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                }
                else if (target_obj == 3)
                {
                    dx(0) = 0;
                    dx(1) = -M_PI_2;
                    dx(2) = 0;
                    dx(3) = b1-0.015;
                    dx(4) = b2;
                    dx(5) = A * sin(f * M_PI * (t - InitTime)) + b3;

                    dx(6) = 0;
                    dx(7) = -M_PI_2;
                    dx(8) = 0;
                    dx(9) = l_p1-0.01;
                    dx(10) = l_p2;
                    dx(11) = -A * sin(f * M_PI * (t - InitTime)) + l_p3;

                    dxdot.setZero();
                    dxdot(5) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(11) = -(f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    dxddot.setZero();
                    dxddot(5) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                    dxddot(11) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                }
                else if( target_obj == 4 )
                {
                    dx(0) = 0;
                    dx(1) = -M_PI_2;
                    dx(2) = 0;
                    dx(3) = A * sin(f * M_PI * (t - InitTime)) + b1-0.015;
                    dx(4) = b2+0.1;
                    dx(5) = b3;

                    dx(6) = 0;
                    dx(7) = 0;
                    dx(8) = 0;
                    dx(9) = -A * cos(2*f * M_PI * (t - InitTime));
                    dx(10) = -A * sin(2*f * M_PI * (t - InitTime)) + 0.45;
                    dx(11) = 0;

                    dxdot.setZero();
                    dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(9) = (2*f * M_PI) * A * sin(2*f * M_PI * (t - InitTime));
                    dxdot(10) = -(2*f * M_PI) * A * cos(2*f * M_PI * (t - InitTime));

                    dxddot.setZero();
                    dxddot(3) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (t - InitTime));
                    dxddot(9) = (2*f * M_PI) * (2*f * M_PI) * A * cos(2*f * M_PI * (t - InitTime));
                    dxddot(10) = (2*f * M_PI) * (2*f * M_PI) * A * sin(2*f * M_PI * (t - InitTime));
                }
            }

            if(t <= InitTime)
            {
                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }
            else
            {
                //Control->TaskInvDynController(dx, dxdot, dxddot, q_.data, qdot_.data, torque, dt, ctrl_type);
                Control->GetControllerStates(qd_.data, qd_dot_.data, ex_);
            }

            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque(i));
            }

            clock_gettime(CLOCK_MONOTONIC, &end);
            // ********* 4. data 저장 *********
            publish_data();

            // ********* 5. state 출력 *********
            print_state();

            t = t + dt;
        }

        void stopping(const ros::Time &time) override
        {
            ROS_INFO("Stop Task-space Controller");
        }

        void manipulability_data()
        {
            const auto desired_manipulability =
                    manipulability_metrics::Ellipsoid{ { { (Eigen::Matrix<double, 6, 1>{} << 1, 0, 0, 0, 0, 0).finished(), 1.0 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 1, 0, 0, 0, 0).finished(), 1.0 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 1, 0, 0, 0).finished(), 1.0 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 1, 0, 0).finished(), 1.0 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 1, 0).finished(), 1.0 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 0, 1).finished(), 1.0 } } };

            const auto desired_Singelmanipulability =
                    manipulability_metrics::Ellipsoid{ { { (Eigen::Matrix<double, 6, 1>{} << 1, 0, 0, 0, 0, 0).finished(), 0.2 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 1, 0, 0, 0, 0).finished(), 0.8 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 1, 0, 0, 0).finished(), 0.8 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 1, 0, 0).finished(), 0.9 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 1, 0).finished(), 1.7 },
                                                               { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 0, 1).finished(), 1.7 } } };
            auto left_ellipsoid = manipulability_metrics::ellipsoidFromJacobian(J2_kdl_.data);
            auto right_ellipsoid = manipulability_metrics::ellipsoidFromJacobian(J1_kdl_.data);
            SingleMM[0] = sqrt((J1_kdl_.data * J1_kdl_.data.transpose()).determinant());
            SingleMM[1] = sqrt((J2_kdl_.data * J2_kdl_.data.transpose()).determinant());
            TOMM[0] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, J1_kdl_.data);
            TOMM[1] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, J2_kdl_.data);
            DAMM = std::max(manipulability_metrics::volumeIntersection(left_ellipsoid, J1_kdl_.data),
                            manipulability_metrics::volumeIntersection(right_ellipsoid, J2_kdl_.data));
            TODAMM2 = manipulability_metrics::dualInverseShapeDiscrepancy(desired_manipulability, J2_kdl_.data, J1_kdl_.data);
        }

        void publish_data()
        {
            static int loop_count_ = 0;
            if(loop_count_ > 2)
            {
                if(state_pub_->trylock())
                {
                    state_pub_->msg_.header.stamp = ros::Time::now();
                    state_pub_->msg_.header.seq++;
                    for(size_t i=0; i<n_joints_; i++)
                    {
                        state_pub_->msg_.q[i] = q_.data(i);
                        state_pub_->msg_.qdot[i] = qdot_.data(i);
                        state_pub_->msg_.dq[i] = qd_.data(i);
                        state_pub_->msg_.dqdot[i] = qd_dot_.data(i);
                        state_pub_->msg_.torque[i] = torque(i);
                    }

                    for(int j=0; j<2; j++)
                    {
                        state_pub_->msg_.dx[j].orientation.x = dx(6*j);
                        state_pub_->msg_.dx[j].orientation.y = dx(6*j+1);
                        state_pub_->msg_.dx[j].orientation.z = dx(6*j+2);
                        state_pub_->msg_.dx[j].position.x = dx(6*j+3);
                        state_pub_->msg_.dx[j].position.y = dx(6*j+4);
                        state_pub_->msg_.dx[j].position.z = dx(6*j+5);

                        state_pub_->msg_.x[j].orientation.x = ForwardOri[j](0);
                        state_pub_->msg_.x[j].orientation.y = ForwardOri[j](1);
                        state_pub_->msg_.x[j].orientation.z = ForwardOri[j](2);
                        state_pub_->msg_.x[j].position.x = ForwardPos[j](0);
                        state_pub_->msg_.x[j].position.y = ForwardPos[j](1);
                        state_pub_->msg_.x[j].position.z = ForwardPos[j](2);

                        state_pub_->msg_.InverseConditionNum[j] = InverseConditionNumber[j];
                    }

                    state_pub_->msg_.DAMM = DAMM;
                    state_pub_->msg_.TODAMM = TODAMM2;
                    state_pub_->unlockAndPublish();
                }
                loop_count_=0;
            }
            loop_count_++;
        }

        void print_state()
        {
            static int count = 0;
            if (count > 499)
            {
                printf("*********************************************************\n\n");
                printf("*** Calcutaion Time (unit: sec)  ***\n");
                printf("t_cal = %0.9lf\n", static_cast<double>(end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_nsec - begin.tv_nsec) / 1000000000.0);
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %0.3lf\n", t);
                printf("CTRL_OBJ:%d\n", target_obj);
                printf("\n");

                printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
                printf("*** States in Joint Space (unit: deg) ***\n");
                for(int i=0; i < n_joints_; i++)
                {
                    printf("[%s]:  \t", joint_names_[i].c_str());
                    printf("q: %0.2lf,\t", q_.data(i) * R2D);
                    printf("qdot: %0.2lf,\t", qdot_.data(i) * R2D);
                    printf("tau: %0.2f\n", torque(i));
                }

                printf("\nForward Kinematics:\n");
                for(int j=0; j<NumChain; j++)
                {
                    printf("no.%d, Actual: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           ForwardPos[j](0), ForwardPos[j](1),ForwardPos[j](2),
                           ForwardOri[j](0), ForwardOri[j](1), ForwardOri[j](2));
                    printf("no.%d, Desired: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           dx(6*j+3), dx(6*j+4),dx(6*j+5), dx(6*j), dx(6*j+1), dx(6*j+2));
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n\n",
                           j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);
                }
                printf("Right e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(0)*RADtoDEG, ex_(1)*RADtoDEG, ex_(2)*RADtoDEG, ex_(3), ex_(4), ex_(5));
                printf("Left e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(6)*RADtoDEG, ex_(7)*RADtoDEG, ex_(8)*RADtoDEG, ex_(9), ex_(10), ex_(11));

                printf("Inverse Condition Number: Right:%0.5lf, Left:%0.5lf \n", InverseConditionNumber[0], InverseConditionNumber[1]);
                printf("SingleMM: Right:%0.5lf, Left:%0.5lf\n", SingleMM[0], SingleMM[1]);
                printf("TOMM: Right:%0.5lf, Left:%0.5lf\n", TOMM[0], TOMM[1]);
                printf("MM: %0.5lf\n", MM);
                printf("DAMM: %0.5lf\n", DAMM);
                printf("TODAMM: %0.5lf\n\n", TODAMM2);
                printf("\n*********************************************************\n");
                count = 0;


                //std::cout << "Weight Damped Jacobian Pseudoinverse:" << std::endl;
                //std::cout << WdampedpInvJac << "\n" << std::endl;
                //std::cout << "lambda:" << std::endl;
                //std::cout << wpInv_lambda[0] << std::endl;
                //std::cout << wpInv_lambda[1] << std::endl;

                //std::cout << "J1:" <<std::endl;
                //std::cout << J1_kdl_.data << "\n"<< std::endl;
                //std::cout << "J2:" <<std::endl;
                //std::cout << J2_kdl_.data << "\n"<< std::endl;
                //std::cout << "Analytic Jacobian:" <<std::endl;
                //std::cout << AJac << "\n"<< std::endl;
                //std::cout << "J1dot:" <<std::endl;
                //std::cout << J1dot.data << "\n"<< std::endl;
                //std::cout << "J2dot:" <<std::endl;
                //std::cout << J2dot.data << "\n"<< std::endl;

                //std::cout << "Body Jacobiandot:" <<std::endl;
                //std::cout << BodyJacDot << "\n"<< std::endl;
                /*
                Jdot.setZero();
                Jdot.block(0,0,3,9) = J1dot.data.block(3,0,3,9);
                Jdot.block(3,0,3,9) = J1dot.data.block(0,0,3,9);
                Jdot.block(6,0,3,2) = J2dot.data.block(3,0,3,2);
                Jdot.block(9,0,3,2) = J2dot.data.block(0,0,3,2);
                Jdot.block(6,9,3,7) = J2dot.data.block(3,2,3,7);
                Jdot.block(9,9,3,7) = J2dot.data.block(0,2,3,7);


                std::cout << "Jdot:" <<std::endl;
                std::cout << Jdot << "\n"<< std::endl;
                std::cout << "Analytic Jacobiandot:" <<std::endl;
                std::cout << AJacDot << "\n"<< std::endl;
                */

            }
            count++;
        }

    private:
        // others
        double t=0.0;
        int target_obj=0;
        int ctrl_type=0;
        double InitTime=0.0;
        struct timespec begin, end;
        //Joint handles
        unsigned int n_joints_;
        std::vector<std::string> joint_names_;
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

        // kdl
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        KDL::Chain kdl_chain2_;

        KDL::JntSpaceInertiaMatrix M_kdl_, M1_kdl_;
        KDL::JntArray C_kdl_, C1_kdl_;
        KDL::JntArray G_kdl_, G1_kdl_;
        KDL::Vector g_kdl_;
        Eigen::VectorXd g_vec_collect;
        Eigen::MatrixXd g_mat_collect;
        Eigen::MatrixXd M_mat_collect;

        KDL::Jacobian J1_kdl_, J2_kdl_;
        KDL::Jacobian J1dot, J2dot;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_, fk_pos_solver1_; //Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_, jnt_to_jac_solver1_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacdot_solver, jnt_to_jacdot_solver1;
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_, id_solver1_;               // Solver To compute the inverse dynamics

        MatrixXd M;
        VectorXd G;
        VectorXd C;

        Vector3d ForwardPos[2];
        Vector3d ForwardOri[2];
        int NumChain=2;
        Vector3d ForwardAxis[2];
        double ForwardAngle[2];
        double InverseConditionNumber[2];
        double SingleMM[2];
        double TOMM[2];
        double MM;
        double DAMM;
        double TODAMM2;

        // kdl and Eigen Jacobian
        Eigen::MatrixXd pInvJac;
        Eigen::MatrixXd AJac, AJacDot, Jdot;
        Eigen::MatrixXd BodyJacDot;

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_;
        KDL::JntArrayVel q1dot_, q2dot_;
        Eigen::VectorXd q0dot;
        Eigen::VectorXd torque;

        // Task Space State
        // ver. 01
        KDL::Frame xd_[2];
        KDL::Frame x_[2];
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::VectorXd ex_;
        Eigen::VectorXd ex_dot_;
        Eigen::VectorXd dx;
        Eigen::VectorXd dxdot;
        Eigen::VectorXd dxddot;

        // Input
        KDL::JntArray x_cmd_;

        // gains
        Eigen::VectorXd KpTask, KdTask;
        Eigen::VectorXd aKp_, aKi_, aKd_, aK_inf_;
        double Kp_trans_R, Kp_rot_R, Kp_trans_L, Kp_rot_L;
        double Kd_trans_R, Kd_rot_R, Kd_trans_L, Kd_rot_L;

        // publisher
        realtime_tools::RealtimeBuffer<std::vector<double>> pub_buffer_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<dualarm_controller::TaskCurrentState>> state_pub_;

        // subscriber
        ros::Subscriber sub_x_cmd_;

        std::shared_ptr<SerialManipulator> cManipulator;
        std::unique_ptr<HYUControl::Controller> Control;
    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::Taskspace_Control,controller_interface::ControllerBase)