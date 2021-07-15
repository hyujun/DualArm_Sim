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
#include <geometry_msgs/WrenchStamped.h>
#include "utils.h"
#include "dualarm_controller/TaskDesiredState.h"
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
#include <Motion.h>

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
    class Impedance_Control : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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

            if (!n.getParam("/dualarm/impedance_control/desired/mass/m1", des_m1))
            {
                ROS_ERROR("Cannot find Right-arm desired mass");
                return false;
            }

            if (!n.getParam("/dualarm/impedance_control/desired/mass/m2", des_m2))
            {
                ROS_ERROR("Cannot find Left-arm desired mass");
                return false;
            }

            if (!n.getParam("/dualarm/impedance_control/gains/mbk/KpT", Kp_trans))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kp Translation proportional gain");
                return false;
            }

            if (!n.getParam("/dualarm/impedance_control/gains/mbk/KpR", Kp_rot))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kp Rotation proportional gain");
                return false;
            }

            if (!n.getParam("/dualarm/impedance_control/gains/mbk/KdT", Kd_trans))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kd Translation derivative gain");
                return false;
            }

            if (!n.getParam("/dualarm/impedance_control/gains/mbk/KdR", Kd_rot))
            {
                ROS_ERROR("Cannot find Left-arm pid/Kd Rotation derivative gain");
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
                ROS_INFO("Found robot_description");
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

            Jdot.setZero(12,n_joints_);

            q1_.resize(9);
            q1dot_.resize(9);
            q2_.resize(9);
            q2dot_.resize(9);

            // ********* 5. 각종 변수 초기화 *********
            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(12);
            ex_.setZero(12);
            ex_dot_.setZero(12);

            dxdot.setZero(12);
            dxddot.setZero(12);
            xdot_logging.setZero(12);

            KpTask.setZero(12);
            KdTask.setZero(12);
            KpNull.setZero(16);
            KdNull.setZero(16);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            torque.setZero(n_joints_);
            ft_sensor.setZero(12);
            torque_ext.setZero(n_joints_);
            des_m.setZero(2);
            targetpos.setZero(12);
            xa.setZero(12);

            wpInv_lambda[0].setZero(3);
            wpInv_lambda[1].setZero(3);


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
                state_pub_->msg_.torque_ext.push_back(torque_ext(i));
            }
            for(int j=0; j<2; j++) {
                state_pub_->msg_.InverseConditionNum.push_back(InverseConditionNumber[j]);
                state_pub_->msg_.SingleMM.push_back(SingleMM[j]);
            }
            state_pub_->msg_.x.resize(2);
            state_pub_->msg_.dx.resize(2);
            state_pub_->msg_.ex.resize(2);
            state_pub_->msg_.DAMM = DAMM;
            state_pub_->msg_.TODAMM = TODAMM2;
            state_pub_->msg_.lambda1.resize(3);
            state_pub_->msg_.lambda2.resize(3);
            state_pub_->msg_.Kp_R = Kp_rot;
            state_pub_->msg_.Kp_T = Kp_trans;
            state_pub_->msg_.ftsensor.resize(12);

            state_pub_->msg_.xdot.resize(12);
            state_pub_->msg_.dxdot.resize(12);
            state_pub_->msg_.dxddot.resize(12);


            pub_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            // 6.2 subsriber
            const auto joint_state_cb = utils::makeCallback<dualarm_controller::TaskDesiredState>([&](const auto& msg){
                ControlIndex1 = msg.Index1;
                ControlIndex2 = msg.Index2;
                ControlSubIndex = msg.SubIndex;
                JointState = ControlSubIndex;
                targetpos(0) = msg.dx[0].orientation.x*DEGtoRAD;
                targetpos(1) = msg.dx[0].orientation.y*DEGtoRAD;
                targetpos(2) = msg.dx[0].orientation.z*DEGtoRAD;
                targetpos(3) = msg.dx[0].position.x;
                targetpos(4) = msg.dx[0].position.y;
                targetpos(5) = msg.dx[0].position.z;
                targetpos(6) = msg.dx[1].orientation.x*DEGtoRAD;
                targetpos(7) = msg.dx[1].orientation.y*DEGtoRAD;
                targetpos(8) = msg.dx[1].orientation.z*DEGtoRAD;
                targetpos(9) = msg.dx[1].position.x;
                targetpos(10) = msg.dx[1].position.y;
                targetpos(11) = msg.dx[1].position.z;
            });
            sub_x_cmd_ = n.subscribe<dualarm_controller::TaskDesiredState>( "command", 5, joint_state_cb);
            sub_ft_sensor_R = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_topic_R", 5, &Impedance_Control::UpdateFTsensorR, this);
            sub_ft_sensor_L = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_topic_L", 5, &Impedance_Control::UpdateFTsensorL, this);
            return true;
        }

        void UpdateFTsensorR(const geometry_msgs::WrenchStamped::ConstPtr &msg)
        {
            geometry_msgs::Wrench ft_measure = msg->wrench;
            ft_sensor(0) = ft_measure.torque.x;
            ft_sensor(1) = ft_measure.torque.y;
            ft_sensor(2) = ft_measure.torque.z;
            ft_sensor(3) = ft_measure.force.x;
            ft_sensor(4) = ft_measure.force.y;
            ft_sensor(5) = ft_measure.force.z;
        }

        void UpdateFTsensorL(const geometry_msgs::WrenchStamped::ConstPtr &msg)
        {
            geometry_msgs::Wrench ft_measure = msg->wrench;
            ft_sensor(6) = ft_measure.torque.x;
            ft_sensor(7) = ft_measure.torque.y;
            ft_sensor(8) = ft_measure.torque.z;
            ft_sensor(9) = ft_measure.force.x;
            ft_sensor(10) = ft_measure.force.y;
            ft_sensor(11) = ft_measure.force.z;
        }

        void starting(const ros::Time &time) override {
            t = 0.0;
            InitTime = 2.0;

            cManipulator = std::make_shared<SerialManipulator>();

            Control = std::make_unique<HYUControl::Controller>(cManipulator);
            motion = std::make_unique<HYUControl::Motion>(cManipulator);

            cManipulator->UpdateManipulatorParam();

            des_m(0) = des_m1;
            des_m(1) = des_m2;

            KpTask.segment(0, 3).setConstant(Kp_rot);
            KpTask.segment(3, 3).setConstant(Kp_trans);
            KpTask.segment(6, 3).setConstant(Kp_rot);
            KpTask.segment(9, 3).setConstant(Kp_trans);

            KdTask.segment(0, 3).setConstant(Kd_rot);
            KdTask.segment(3, 3).setConstant(Kd_trans);
            KdTask.segment(6, 3).setConstant(Kd_rot);
            KdTask.segment(9, 3).setConstant(Kd_trans);

            KpNull.setConstant(16,0.001);
            KdNull.setConstant(16,0.4 );
            Control->SetImpedanceGain(KpTask, KdTask, KpNull, KdNull, des_m);

            ControlIndex1 = CTRLMODE_IDY_JOINT;
            ControlIndex2 = SYSTEM_BEGIN;
            ControlSubIndex = MOVE_ZERO;
            JointState = MOVE_ZERO;

            ROS_INFO("Starting Impedance Controller");

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
            cManipulator->pKin->GetAnalyticJacobian(AJac);

            xdot_logging = AJac*qdot_.data;

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

            cManipulator->pDyn->M_Matrix(M);
            KineticEnergy = qdot_.data.transpose()*M*qdot_.data;

            manipulability_data();

            ctrl_type = ControlIndex2;
            target_obj = ControlSubIndex;

            if( ControlIndex1 == CTRLMODE_IMPEDANCE_TASK )
            {
                if(ControlIndex2 == 3)
                {
                    cManipulator->pKin->GetForwardKinematicsWithRelative(xa);
                    cManipulator->pKin->GetRelativeJacobian(RelativeJac);
                    SingleMM[0] = cManipulator->pKin->GetManipulabilityMeasure(AJac.block(0,0,6,16));
                    SingleMM[1] = cManipulator->pKin->GetManipulabilityMeasure(RelativeJac);
                    AJac.block(6,0,6,16) = RelativeJac;
                    MM = cManipulator->pKin->GetManipulabilityMeasure(AJac);
                }
                else
                {
                    cManipulator->pKin->GetForwardKinematics(xa);
                    SingleMM[0] = cManipulator->pKin->GetManipulabilityMeasure(AJac.block(0,0,6,16));
                    SingleMM[1] = cManipulator->pKin->GetManipulabilityMeasure(AJac.block(6,0,6,16));
                    MM = cManipulator->pKin->GetManipulabilityMeasure();
                }
                motion->TaskMotion(dx, dxdot, dxddot, targetpos, xa, qdot_.data, t, JointState, ControlSubIndex);
                Control->TaskImpedanceController(q_.data, qdot_.data, dx, dxdot, dxddot, ft_sensor, torque, ControlIndex2);
                Control->GetControllerStates(qd_.data, qd_dot_.data, ex_);
                cManipulator->pKin->GetWDampedpInvLambda(wpInv_lambda);
            }
            else if( ControlIndex1 == CTRLMODE_IDY_JOINT )
            {
                MM = cManipulator->pKin->GetManipulabilityMeasure();
                qd_dot_.data.setZero(16);
                motion->JointMotion(qd_.data, qd_dot_.data, qd_ddot_.data, targetpos, q_.data, qdot_.data, t, JointState, ControlSubIndex);
                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
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
            ROS_INFO("Stop Impedance Controller");
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
                    torque_ext = AJac.transpose()*ft_sensor;
                    for(size_t i=0; i<n_joints_; i++)
                    {
                        state_pub_->msg_.q[i] = q_.data(i);
                        state_pub_->msg_.qdot[i] = qdot_.data(i);
                        state_pub_->msg_.dq[i] = qd_.data(i);
                        state_pub_->msg_.dqdot[i] = qd_dot_.data(i);
                        state_pub_->msg_.torque[i] = torque(i);
                        state_pub_->msg_.torque_ext[i] = torque_ext(i);
                    }

                    Quaterniond tmp;

                    for(int j=0; j<2; j++)
                    {
                        tmp = dx[j].r;
                        state_pub_->msg_.dx[j].orientation.x = tmp.x();
                        state_pub_->msg_.dx[j].orientation.y = tmp.y();
                        state_pub_->msg_.dx[j].orientation.z = tmp.z();
                        state_pub_->msg_.dx[j].position.x = dx[j].p(0);
                        state_pub_->msg_.dx[j].position.y = dx[j].p(1);
                        state_pub_->msg_.dx[j].position.z = dx[j].p(2);

                        state_pub_->msg_.x[j].orientation.x = xa(6*j);
                        state_pub_->msg_.x[j].orientation.y = xa(6*j+1);
                        state_pub_->msg_.x[j].orientation.z = xa(6*j+2);
                        state_pub_->msg_.x[j].position.x = xa(6*j+3);
                        state_pub_->msg_.x[j].position.y = xa(6*j+4);
                        state_pub_->msg_.x[j].position.z = xa(6*j+5);

                        state_pub_->msg_.ex[j].orientation.x = ex_(6*j);
                        state_pub_->msg_.ex[j].orientation.y = ex_(6*j+1);
                        state_pub_->msg_.ex[j].orientation.z = ex_(6*j+2);
                        state_pub_->msg_.ex[j].position.x = ex_(6*j+3);
                        state_pub_->msg_.ex[j].position.y = ex_(6*j+4);
                        state_pub_->msg_.ex[j].position.z = ex_(6*j+5);

                        state_pub_->msg_.InverseConditionNum[j] = InverseConditionNumber[j];
                        state_pub_->msg_.SingleMM[j] = SingleMM[j];
                    }

                    state_pub_->msg_.MM = MM;
                    state_pub_->msg_.DAMM = DAMM;
                    state_pub_->msg_.TODAMM = TODAMM2;
                    state_pub_->msg_.lambda1[0] = wpInv_lambda[0](0);
                    state_pub_->msg_.lambda1[1] = wpInv_lambda[0](1);
                    state_pub_->msg_.lambda1[2] = wpInv_lambda[0](2);
                    state_pub_->msg_.lambda2[0] = wpInv_lambda[1](0);
                    state_pub_->msg_.lambda2[1] = wpInv_lambda[1](1);
                    state_pub_->msg_.lambda2[2] = wpInv_lambda[1](2);
                    for(int k=0; k<12; k++)
                    {
                        state_pub_->msg_.ftsensor[k] = ft_sensor(k);
                        state_pub_->msg_.xdot[k] = xdot_logging(k);
                        state_pub_->msg_.dxdot[k] = dxdot(k);
                        state_pub_->msg_.dxddot[k] = dxddot(k);
                    }
                    state_pub_->msg_.KE = KineticEnergy;

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
                printf("Index1:%d\n", ControlIndex1);
                printf("Index2:%d\n", ControlIndex2);
                printf("SubIndex:%d\n", ControlSubIndex);
                printf("MsgSubIndex:%d\n", JointState);
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
                    double a, b, g;
                    x_[j].M.GetEulerZYX(a, b, g);
                    printf("no.%d, DH: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n",
                           j, x_[j].p(0), x_[j].p(1),x_[j].p(2), g, b, a);
                    Vector3d RTmp;
                    RTmp = dx[j].r.eulerAngles(2,1,0);
                    printf("no.%d, Desired: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           dx[j].p(0), dx[j].p(1), dx[j].p(2), RTmp(0), RTmp(1), RTmp(2));
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n\n",
                           j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);
                }
                printf("Right e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(0)*RADtoDEG, ex_(1)*RADtoDEG, ex_(2)*RADtoDEG, ex_(3), ex_(4), ex_(5));
                printf("Left e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n\n",
                       ex_(6)*RADtoDEG, ex_(7)*RADtoDEG, ex_(8)*RADtoDEG, ex_(9), ex_(10), ex_(11));

                printf("FT Sensor(Right): torque_u:%0.3lf, torque_v:%0.3lf, torque_w:%0.3lf, force_x:%0.3lf, force_y:%0.3lf, force_z:%0.3lf\n",
                       ft_sensor(0), ft_sensor(1), ft_sensor(2),ft_sensor(3),ft_sensor(4),ft_sensor(5));
                printf("FT Sensor(Left): torque_u:%0.3lf, torque_v:%0.3lf, torque_w:%0.3lf, force_x:%0.3lf, force_y:%0.3lf, force_z:%0.3lf\n\n",
                       ft_sensor(6), ft_sensor(7), ft_sensor(8),ft_sensor(9),ft_sensor(10),ft_sensor(11));

                printf("Inverse Condition Number: Right:%0.5lf, Left:%0.5lf \n", InverseConditionNumber[0], InverseConditionNumber[1]);
                printf("SingleMM: Right:%0.5lf, Left:%0.5lf\n", SingleMM[0], SingleMM[1]);
                printf("TOMM: Right:%0.5lf, Left:%0.5lf\n", TOMM[0], TOMM[1]);
                printf("MM: %0.5lf\n", MM);
                printf("DAMM: %0.5lf\n", DAMM);
                printf("TODAMM: %0.5lf\n", TODAMM2);
                printf("*********************************************************\n");
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

        unsigned char ControlIndex1;
        unsigned char ControlIndex2;
        unsigned char ControlSubIndex;
        unsigned char JointState;

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
        double KineticEnergy;

        // kdl and Eigen Jacobian
        Eigen::MatrixXd pInvJac;
        Eigen::MatrixXd AJac, AJacDot, Jdot;
        Eigen::MatrixXd RelativeJac;
        Eigen::MatrixXd BodyJacDot;

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_;
        KDL::JntArrayVel q1dot_, q2dot_;
        Eigen::VectorXd xa;
        Eigen::VectorXd xdot_logging;
        Eigen::VectorXd q0dot;
        Eigen::VectorXd torque;
        Eigen::VectorXd ft_sensor;
        Eigen::VectorXd torque_ext;
        Eigen::VectorXd targetpos;
        Eigen::VectorXd wpInv_lambda[2];

        // Task Space State
        // ver. 01
        KDL::Frame xd_[2];
        KDL::Frame x_[2];
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::VectorXd ex_;
        Eigen::VectorXd ex_dot_;
        Cartesiand dx[2];
        Eigen::VectorXd dxdot;
        Eigen::VectorXd dxddot;

        // Input
        KDL::JntArray x_cmd_;

        // gains
        Eigen::VectorXd KpTask, KdTask, des_m;
        Eigen::VectorXd KpNull, KdNull;
        Eigen::VectorXd aKp_, aKi_, aKd_, aK_inf_;
        double des_m1, des_m2;
        double Kp_trans, Kp_rot, Kd_trans, Kd_rot;

        // publisher
        realtime_tools::RealtimeBuffer<std::vector<double>> pub_buffer_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<dualarm_controller::TaskCurrentState>> state_pub_;

        // subscriber
        ros::Subscriber sub_x_cmd_;
        ros::Subscriber sub_ft_sensor_R, sub_ft_sensor_L;

        std::shared_ptr<SerialManipulator> cManipulator;
        std::unique_ptr<HYUControl::Controller> Control;
        std::unique_ptr<HYUControl::Motion> motion;
    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::Impedance_Control,controller_interface::ControllerBase)