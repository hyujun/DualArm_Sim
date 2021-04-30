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
#include "dualarm_controller/TaskDesiredState.h"

//manipulability
#include <ellipsoid.h>
#include <similarity.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <boost/scoped_ptr.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include <SerialManipulator.h>
#include <Controller.h>
#include <Motion.h>

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

#define A 0.12
#define b1 0.55
#define b2 -0.43
#define b3 0.45
#define f 0.2

#define l_p1 0.45
#define l_p2 0.43
#define l_p3 0.39

#define Deg_A 70
#define Deg_f 0.5

namespace  dualarm_controller
{
    class ClosedLoopIK_Control : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // ********* 1. Get joint name / gain from the parameter server *********
            // 1.0 Control objective & Inverse Kinematics mode
            if (!n.getParam("ctr_obj", ctr_obj_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            if (!n.getParam("ik_mode", ik_mode_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            if( ctr_obj_ == 7 && ik_mode_ != 4 )
            {
                ROS_ERROR("!! If the ctr_obj is 8, ik_mode_ should be 4 !!");
                return false;
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
            // 1.2.1 Joint Controller
            Kp_.data.setZero(n_joints_);
            Kd_.data.setZero(n_joints_);
            Ki_.data.setZero(n_joints_);
            K_inf_.data.setZero(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/p", Kp_.data(i)))
                {
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/i", Ki_.data(i)))
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/d", Kd_.data(i)))
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/h", K_inf_.data(i)))
                {
                    ROS_ERROR("Cannot find pid/h gain");
                    return false;
                }
            }

            // 1.2.2 Closed-loop Inverse Kinematics Controller

            if (!n.getParam("/dualarm/closedloopik_control/clik_gain/K_pos", K_trans))
            {
                ROS_ERROR("Cannot find clik translation gain");
                return false;
            }

            if (!n.getParam("/dualarm/closedloopik_control/clik_gain/K_ori", K_rot))
            {
                ROS_ERROR("Cannot find clik rotation gain");
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
            else
            {
                ROS_INFO_STREAM("Constructed kdl tree");
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
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            J1_kdl_.resize(kdl_chain_.getNrOfJoints());
            M_kdl_.resize(kdl_chain_.getNrOfJoints());
            C_kdl_.resize(kdl_chain_.getNrOfJoints());
            G_kdl_.resize(kdl_chain_.getNrOfJoints());

            id_solver1_.reset(new KDL::ChainDynParam(kdl_chain2_, g_kdl_));
            jnt_to_jac_solver1_.reset(new KDL::ChainJntToJacSolver(kdl_chain2_));
            fk_pos_solver1_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain2_));
            J2_kdl_.resize(kdl_chain2_.getNrOfJoints());
            M1_kdl_.resize(kdl_chain2_.getNrOfJoints());
            C1_kdl_.resize(kdl_chain2_.getNrOfJoints());
            G1_kdl_.resize(kdl_chain2_.getNrOfJoints());

            // ********* 5. 각종 변수 초기화 *********
            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(12);
            targetpos.setZero(12);
            ex_.setZero(12);
            ex_dot_.setZero(12);
            dx.setZero(12);
            dxdot.setZero(12);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            torque.setZero(n_joints_);

            wpInv_lambda[0].setZero(3);
            wpInv_lambda[1].setZero(3);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            state_pub_.reset(new realtime_tools::RealtimePublisher<dualarm_controller::TaskCurrentState>(n, "states", 10));
            state_pub_->msg_.header.stamp = ros::Time::now();
            state_pub_->msg_.header.frame_id = "dualarm";
            state_pub_->msg_.header.seq=0;
            for(int i=0; i<(n_joints_-1); i++)
            {
                state_pub_->msg_.q.push_back(q_.data(i));
                state_pub_->msg_.qdot.push_back(qdot_.data(i));
                state_pub_->msg_.dq.push_back(qd_.data(i));
                state_pub_->msg_.dqdot.push_back(qd_dot_.data(i));
                state_pub_->msg_.torque.push_back(torque(i));
            }
            for(int j=0; j<2; j++) {
                state_pub_->msg_.InverseConditionNum.push_back(InverseConditionNumber[j]);
                state_pub_->msg_.SingleMM.push_back(SingleMM[j]);
            }
            state_pub_->msg_.x.resize(2);
            state_pub_->msg_.dx.resize(2);
            state_pub_->msg_.MM = MM;
            state_pub_->msg_.DAMM = DAMM;
            state_pub_->msg_.TODAMM = TODAMM2;
            state_pub_->msg_.lambda1.resize(3);
            state_pub_->msg_.lambda2.resize(3);
            state_pub_->msg_.Kp_R = K_rot;
            state_pub_->msg_.Kp_T = K_trans;

            pub_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            // 6.2 subsriber
            const auto joint_state_cb = utils::makeCallback<dualarm_controller::TaskDesiredState>([&](const auto& msg){
                ControlMode = msg.Index1;
                ControlSubMode = msg.Index2;
                ControlMotion = msg.SubIndex;
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

            return true;
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            InitTime=1.0;

            ROS_INFO("Starting Closed-loop Inverse Dynamics Controller");

            cManipulator = std::make_shared<SerialManipulator>();

            Control = std::make_unique<HYUControl::Controller>(cManipulator);
            motion = std::make_unique<HYUControl::Motion>(cManipulator);

            cManipulator->UpdateManipulatorParam();

            CLIK_GAIN_DUMY.setZero(12);
            CLIK_GAIN.setZero(12);
            CLIK_GAIN(0) = K_rot;
            CLIK_GAIN(1) = K_rot;
            CLIK_GAIN(2) = K_rot;
            CLIK_GAIN(3) = K_trans;
            CLIK_GAIN(4) = K_trans;
            CLIK_GAIN(5) = K_trans;
            CLIK_GAIN(6) = K_rot;
            CLIK_GAIN(7) = K_rot;
            CLIK_GAIN(8) = K_rot;
            CLIK_GAIN(9) = K_trans;
            CLIK_GAIN(10) = K_trans;
            CLIK_GAIN(11) = K_trans;

            Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data, K_inf_.data);
            Control->SetTaskspaceGain(CLIK_GAIN, CLIK_GAIN_DUMY);
            alpha = 5.0;

            ControlMode = CTRLMODE_IDY_JOINT;
            ControlSubMode = SYSTEM_BEGIN;
            ControlMotion = MOVE_ZERO;
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
            cManipulator->pKin->GetAnalyticJacobian(AJac);
            cManipulator->pKin->GetpinvJacobian(pInvJac);

            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);
            cManipulator->pKin->GetAngleAxis(ForwardAxis, ForwardAngle, NumChain);
            cManipulator->pKin->GetInverseConditionNumber(InverseConditionNumber);

            q1_.data = q_.data.head(9);
            q1dot_.data = qdot_.data.head(9);

            q2_.resize(9);
            q2dot_.resize(9);
            q2_.data.tail(7) = q_.data.tail(7);
            q2_.data.head(2) = q_.data.head(2);
            q2dot_.data = qdot_.data.tail(7);
            q2dot_.data.head(2) = qdot_.data.head(2);

            fk_pos_solver_->JntToCart(q1_, x_[0]);
            fk_pos_solver1_->JntToCart(q2_, x_[1]);

            jnt_to_jac_solver_->JntToJac(q1_, J1_kdl_);
            jnt_to_jac_solver1_->JntToJac(q2_, J2_kdl_);

            MM = cManipulator->pKin->GetManipulabilityMeasure();
            manipulability_data();

            //cManipulator->pDyn->MG_Mat_Joint(M, G);
            //id_solver_->JntToMass(q1_, M_kdl_);
            //id_solver_->JntToCoriolis(q1_, q1dot_, C_kdl_);
            //id_solver_->JntToGravity(q1_, G_kdl_);
            //id_solver1_->JntToMass(q2_, M1_kdl_);
            //id_solver1_->JntToCoriolis(q2_, q2dot_, C1_kdl_);
            //id_solver1_->JntToGravity(q2_, G1_kdl_);

            ctr_obj_ = ControlSubMode;
            ik_mode_ = ControlMotion;

            if( ControlMode == CTRLMODE_CLIK )
            {
                motion->TaskMotion(dx, dxdot, dxddot, targetpos, q_.data, qdot_.data, t, JointState, ControlMotion);
                Control->CLIKTaskController(q_.data, qdot_.data, dx, dxdot, torque, dt, ControlSubMode);
                Control->GetControllerStates(qd_.data, qd_dot_.data, ex_);

                if(ctr_obj_ == 6)
                    cManipulator->pKin->GetWDampedpInvLambda(wpInv_lambda);
            }
            else if( ControlMode == CTRLMODE_IDY_JOINT )
            {
                qd_dot_.data.setZero(16);
                motion->JointMotion(qd_.data, qd_dot_.data, qd_ddot_.data, targetpos, q_.data, qdot_.data, t, JointState, ControlMotion);
                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }

            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque(i));
            }

            clock_gettime(CLOCK_MONOTONIC, &end);
            // ********* 4. data 저장 *********
            if(t > InitTime)
                publish_data();

            // ********* 5. state 출력 *********
            print_state();

            t = t + dt;
        }

        void stopping(const ros::Time &time) override
        {
            ROS_INFO("Stop Closed-loop Inverse Dynamics Controller");
        }

        void manipulability_data()
        {
            const auto desired_Dualmanipulability =
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

            SingleMM[0] = sqrt((J1_kdl_.data * J1_kdl_.data.transpose()).determinant());
            SingleMM[1] = sqrt((J2_kdl_.data * J2_kdl_.data.transpose()).determinant());

            SingleMM_1[0] = sqrt((AJac.block(0,0,6,16)*AJac.block(0,0,6,16).transpose()).determinant());
            SingleMM_1[1] = sqrt((AJac.block(6,0,6,16)*AJac.block(6,0,6,16).transpose()).determinant());

            TOMM[0] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, J1_kdl_.data);
            TOMM[1] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, J2_kdl_.data);

            TOMM_1[0] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, AJac.block(0,0,6,16));
            TOMM_1[1] = manipulability_metrics::inverseShapeDiscrepancy(desired_Singelmanipulability, AJac.block(6,0,6,16));

            auto left_ellipsoid = manipulability_metrics::ellipsoidFromJacobian(J2_kdl_.data);
            auto right_ellipsoid = manipulability_metrics::ellipsoidFromJacobian(J1_kdl_.data);
            DAMM = std::max(manipulability_metrics::volumeIntersection(left_ellipsoid, J1_kdl_.data),
                                 manipulability_metrics::volumeIntersection(right_ellipsoid, J2_kdl_.data));
            TODAMM2 = manipulability_metrics::dualInverseShapeDiscrepancy(desired_Dualmanipulability, J2_kdl_.data, J1_kdl_.data);

            auto left_ellipsoid_1 = manipulability_metrics::ellipsoidFromJacobian(AJac.block(6,0,6,16));
            auto right_ellipsoid_1 = manipulability_metrics::ellipsoidFromJacobian(AJac.block(0,0,6,16));
            DAMM_1 = std::max(manipulability_metrics::volumeIntersection(left_ellipsoid_1, AJac.block(0,0,6,16)),
                            manipulability_metrics::volumeIntersection(right_ellipsoid_1, AJac.block(6,0,6,16)));
            TODAMM2_1 = manipulability_metrics::dualInverseShapeDiscrepancy(desired_Dualmanipulability, AJac.block(6,0,6,16), AJac.block(0,0,6,16));
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
                        state_pub_->msg_.SingleMM[j] = SingleMM[j];

                    }
                    state_pub_->msg_.MM = MM;
                    state_pub_->msg_.DAMM = DAMM;
                    state_pub_->msg_.TODAMM = TODAMM2;
                    state_pub_->msg_.Kp_T = K_trans;
                    state_pub_->msg_.Kp_R = K_rot;
                    state_pub_->msg_.lambda1[0] = wpInv_lambda[0](0);
                    state_pub_->msg_.lambda1[1] = wpInv_lambda[0](1);
                    state_pub_->msg_.lambda1[2] = wpInv_lambda[0](2);
                    state_pub_->msg_.lambda2[0] = wpInv_lambda[1](0);
                    state_pub_->msg_.lambda2[1] = wpInv_lambda[1](1);
                    state_pub_->msg_.lambda2[2] = wpInv_lambda[1](2);
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
                printf("IK mode:%d, CTRL_OBJ:%d\n", ik_mode_, ctr_obj_);
                printf("\n");

                printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
                printf("*** States in Joint Space (unit: deg) ***\n");
                Control->GetPIDGain(aKp_, aKd_, aKi_);
                for(int i=0; i < n_joints_; i++)
                {
                    printf("[%s]:  \t", joint_names_[i].c_str());
                    printf("Kp:%0.3lf, Kd:%0.3lf,\t", aKp_(i), aKd_(i));
                    printf("q: %0.3lf,\t", q_.data(i) * R2D);
                    printf("dq: %0.3lf,\t", qd_.data(i) * R2D);
                    printf("qdot: %0.3lf,\t", qdot_.data(i) * R2D);
                    printf("dqdot: %0.3lf,\t", qd_dot_.data(i) * R2D);
                    printf("tau: %0.3f\n", torque(i));
                }

                printf("\nForward Kinematics:\n");
                for(int j=0; j<NumChain; j++)
                {
                    printf("no.%d, PoE: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           ForwardPos[j](0), ForwardPos[j](1),ForwardPos[j](2),
                           ForwardOri[j](0), ForwardOri[j](1), ForwardOri[j](2));
                    printf("no.%d, PoE_desired: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           dx(6*j+3),dx(6*j+4),dx(6*j+5),dx(6*j), dx(6*j+1), dx(6*j+2));
                    double a, b, g;
                    x_[j].M.GetEulerZYX(a, b, g);
                    printf("no.%d, DH: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n",
                           j, x_[j].p(0), x_[j].p(1),x_[j].p(2), g, b, a);
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n",
                           j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);
                    printf("\n");
                }
                printf("Inverse Condition Number: Right:%0.5lf, Left:%0.5f\n", InverseConditionNumber[0], InverseConditionNumber[1]);
                printf("SingleMM: Right:%0.5lf, Left:%0.5lf :: Right:%0.5lf, Left:%0.5lf,\n", SingleMM[0], SingleMM[1], SingleMM_1[0], SingleMM_1[1]);
                printf("TOMM: Right:%0.5lf, Left:%0.5lf :: Right:%0.5lf, Left:%0.5lf\n", TOMM[0], TOMM[1], TOMM_1[0], TOMM_1[1]);
                printf("MM: %0.5lf\n", MM);
                printf("DAMM: %0.5lf :: DAMM: %0.5lf\n", DAMM, DAMM_1);
                printf("TODAMM: %0.5lf :: TODAMM: %0.5lf\n\n", TODAMM2, TODAMM2_1);

                printf("Right e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(0)*RADtoDEG, ex_(1)*RADtoDEG, ex_(2)*RADtoDEG, ex_(3), ex_(4), ex_(5));
                printf("Left e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(6)*RADtoDEG, ex_(7)*RADtoDEG, ex_(8)*RADtoDEG, ex_(9), ex_(10), ex_(11));
                printf("\n*********************************************************\n");
                count = 0;

                if(ctr_obj_ == 6)
                {
                    std::cout << "lambda[right]:" << std::endl;
                    std::cout << wpInv_lambda[0] << std::endl;
                    std::cout << "lambda[left]:" << std::endl;
                    std::cout << wpInv_lambda[1] << std::endl;
                }

                //std::cout << "J1:" <<std::endl;
                //std::cout << J1_kdl_.data << "\n"<< std::endl;
                //std::cout << "J2:" <<std::endl;
                //std::cout << J2_kdl_.data << "\n"<< std::endl;
                //std::cout << "Analytic Jacobian:" <<std::endl;
                //std::cout << AJac << "\n"<< std::endl;
            }
            count++;
        }

    private:
        // others
        double t=0.0;
        int ctr_obj_=0;
        int ik_mode_=0;
        double InitTime=0.0;

        unsigned char ControlMode;
        unsigned char ControlSubMode;
        unsigned char ControlMotion;
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

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_, fk_pos_solver1_; //Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_, jnt_to_jac_solver1_; //Solver to compute the jacobian
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
        double SingleMM[2], SingleMM_1[2];
        double TOMM[2], TOMM_1[2];
        double MM, MM_1;
        double DAMM, DAMM_1;
        double TODAMM2, TODAMM2_1;

        // kdl and Eigen Jacobian
        Eigen::MatrixXd pInvJac;
        Eigen::MatrixXd AJac;
        Eigen::MatrixXd BlockpInvJac;
        Eigen::MatrixXd ScaledJac;
        Eigen::MatrixXd dampedpInvJac;
        Eigen::MatrixXd WdampedpInvJac;
        VectorXd wpInv_lambda[2];

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_, q1dot_, q2dot_;
        Eigen::VectorXd q0dot;
        double alpha;
        Eigen::VectorXd torque;
        Eigen::VectorXd targetpos;

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
        KDL::JntArray Kp_, Ki_, Kd_, K_inf_;
        Eigen::VectorXd aKp_, aKi_, aKd_, aK_inf_;
        double K_trans, K_rot;
        Eigen::VectorXd CLIK_GAIN, CLIK_GAIN_DUMY;

        // publisher
        realtime_tools::RealtimeBuffer<std::vector<double>> pub_buffer_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<dualarm_controller::TaskCurrentState>> state_pub_;

        // subscriber
        ros::Subscriber sub_x_cmd_;

        std::shared_ptr<SerialManipulator> cManipulator;
        std::unique_ptr<HYUControl::Controller> Control;
        std::unique_ptr<HYUControl::Motion> motion;
    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::ClosedLoopIK_Control,controller_interface::ControllerBase)