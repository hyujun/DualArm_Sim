//
// Created by june on 19. 12. 5..
//

// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>
#define _USE_MATH_DEFINES

#include <SerialManipulator.h>
#include <Controller.h>

#define D2R M_PI/180.0
#define R2D 180.0/M_PI
#define num_taskspace 6
#define SaveDataMax 97

#define A 0.12
#define b1 0.6
#define b2 -0.43
#define b3 0.45
#define f 0.2

#define l_p1 0.4
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
                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/p", Kp_(i)))
                {
                    std::cout << "/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/i", Ki_(i)))
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/d", Kd_(i)))
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }

                if (!n.getParam("/dualarm/closedloopik_control/gains/dualarm_joint" + si + "/pid/h", K_inf_(i)))
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
            else
            {
                ROS_INFO("Constructed kdl tree");
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
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else if(!kdl_tree_.getChain(root_name, tip_name2, kdl_chain2_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name2);
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO("Got kdl chain");
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

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher

            // 6.2 subsriber
            sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>(
                    "command",
                    1, &ClosedLoopIK_Control::commandCB,
                    this);

            return true;
        }


        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if (msg->data.size() != 2*num_taskspace)
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
                return;
            }

            for (int i = 0; i < 2*num_taskspace; i++)
            {
                x_cmd_(i) = msg->data[i];
            }
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            InitTime=5.0;

            ROS_INFO("Starting Closed-loop Inverse Dynamics Controller");

            cManipulator = std::make_shared<SerialManipulator>();

            Control = std::make_unique<HYUControl::Controller>(cManipulator);

            cManipulator->UpdateManipulatorParam();

            Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data, K_inf_.data);
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

        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();

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
            cManipulator->pDyn->MG_Mat_Joint(M, G);

            cManipulator->pKin->GetpinvJacobian(pInvJac);
            cManipulator->pKin->GetAnalyticJacobian(AJac);
            cManipulator->pKin->GetScaledTransJacobian(ScaledTransJac);
            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);
            cManipulator->pKin->GetAngleAxis(ForwardAxis, ForwardAngle, NumChain);
            cManipulator->pKin->GetInverseConditionNumber(InverseConditionNumber);
            DAManipulabilityMeasure = cManipulator->pKin->GetDAManipulabilityMeasure();

            q1_.data = q_.data.head(9);
            q1dot_.data = qdot_.data.head(9);
            jnt_to_jac_solver_->JntToJac(q1_, J1_kdl_);
            id_solver_->JntToMass(q1_, M_kdl_);
            id_solver_->JntToCoriolis(q1_, q1dot_, C_kdl_);
            id_solver_->JntToGravity(q1_, G_kdl_);
            fk_pos_solver_->JntToCart(q1_, x_[0]);

            q2_.resize(9);
            q2dot_.resize(9);
            q2_.data.tail(7) = q_.data.tail(7);
            q2_.data.head(2) = q_.data.head(2);
            q2dot_.data = qdot_.data.tail(7);
            q2dot_.data.head(2) = qdot_.data.head(2);
            jnt_to_jac_solver1_->JntToJac(q2_, J2_kdl_);
            id_solver1_->JntToMass(q2_, M1_kdl_);
            id_solver1_->JntToCoriolis(q2_, q2dot_, C1_kdl_);
            id_solver1_->JntToGravity(q2_, G1_kdl_);
            fk_pos_solver1_->JntToCart(q2_, x_[1]);

            if( t <= InitTime || ctr_obj_ == 0 )
            {
                qd_.data.setZero();

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

                qd_old_ = qd_;
            }
            else if ( ctr_obj_ >= 1 && ctr_obj_ <= 3 )
            {
                if (ik_mode_ == 1)
                {
                    xd_[0].p(0) = A * sin(f * M_PI * (t - InitTime)) + b1;
                    xd_[0].p(1) = b2;
                    xd_[0].p(2) = b3;
                    xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(0, 0, M_PI/2));

                    xd_[1].p(0) = A * sin(f * M_PI * (t - InitTime)) + l_p1;
                    xd_[1].p(1) = l_p2;
                    xd_[1].p(2) = l_p3;
                    xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(-M_PI/2, 0, M_PI/2));

                    dxdot.setZero();
                    dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(9) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

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
                else if (ik_mode_ == 2)
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
                    dx(10) = A * sin(f * M_PI * (t - InitTime)) + l_p2;
                    dx(11) = l_p3;

                    dxdot.setZero();
                    dxdot(4) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(10) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    Control->TaskError(dx, dxdot, qdot_.data, ex_, ex_dot_);
                }
                else if (ik_mode_ == 3)
                {

                    dx(0) = 0;
                    dx(1) = -M_PI_2;
                    dx(2) = 0;
                    dx(3) = b1-0.01;
                    dx(4) = b2;
                    dx(5) = A * sin(f * M_PI * (t - InitTime)) + b3;

                    dx(6) = 0;
                    dx(7) = -M_PI_2;
                    dx(8) = 0;
                    dx(9) = l_p1-0.01;
                    dx(10) = l_p2;
                    dx(11) = A * sin(f * M_PI * (t - InitTime)) + l_p3;

                    dxdot.setZero();
                    dxdot(5) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    dxdot(11) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    Control->TaskError(dx, dxdot, qdot_.data, ex_, ex_dot_);
                }
            }
            else if( ctr_obj_ == 4 )
            {
                dx(0) = 0;
                dx(1) = -M_PI_2;
                dx(2) = 0;
                dx(3) = b1;
                dx(4) = b2;
                dx(5) = b3;

                dx(6) = 0;
                dx(7) = -M_PI_2;
                dx(8) = 0; //M_PI/2;
                dx(9) = l_p1;
                dx(10) = l_p2;
                dx(11) = l_p3;

                dxdot.setZero();
                //dxdot(5) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                //dxdot(11) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                Control->TaskError(dx, dxdot, qdot_.data, ex_, ex_dot_);

            }

            if( ctr_obj_ == 4 && t > InitTime)
            {
                Control->TaskInvDynController(ex_, ex_dot_, q_.data, qdot_.data, torque, dt);
            }
            else if( ctr_obj_ == 3 && t > InitTime )
            {
                qd_ddot_.data.setZero();
                qd_dot_.data.setZero();
                q0dot.setZero(16);

                double alpha = 1.0;
                cManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
                qd_dot_.data = pInvJac * (dxdot + CLIK_GAIN.cwiseProduct(ex_)) + (Eigen::MatrixXd::Identity(16,16) - pInvJac*AJac)*q0dot;

                qd_.data = qd_old_.data + qd_dot_.data * dt;
                qd_old_.data = qd_.data;

                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }
            else if( ctr_obj_ == 2 && t > InitTime )
            {
                qd_ddot_.data.setZero();
                qd_dot_.data.setZero();
                q0dot.setZero(16);

                double alpha = 1.0;
                cManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
                qd_dot_.data = AJac.transpose() * (dxdot + CLIK_GAIN.cwiseProduct(ex_)) + (Eigen::MatrixXd::Identity(16,16) - pInvJac*AJac)*q0dot;

                qd_.data = qd_old_.data + qd_dot_.data * dt;
                qd_old_.data = qd_.data;

                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }
            else if( ctr_obj_ == 1 && t > InitTime )
            {
                qd_ddot_.data.setZero();
                qd_dot_.data.setZero();
                q0dot.setZero(16);

                double alpha = 1.0;
                cManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
                qd_dot_.data = ScaledTransJac * (dxdot + CLIK_GAIN.cwiseProduct(ex_)) + (Eigen::MatrixXd::Identity(16,16) - AJac.transpose()*AJac)*q0dot;

                qd_.data = qd_old_.data + qd_dot_.data * dt;
                qd_old_.data = qd_.data;

                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }
            else
            {
                qd_ddot_.data.setZero();
                qd_dot_.data.setZero();
                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }

            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque(i));
                //joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            publish_data();

            // ********* 5. state 출력 *********
            print_state();

            t = t + dt;
        }

        void stopping(const ros::Time &time) override
        {
            ROS_INFO("Stop Closed-loop Inverse Dynamics Controller");
        }

        void publish_data()
        {

        }

        void print_state()
        {
            static int count = 0;
            if (count > 499)
            {
                printf("*********************************************************\n\n");
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %f\n", t);
                printf("\n");

                printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
                printf("*** States in Joint Space (unit: deg) ***\n");
                Control->GetPIDGain(Kp_.data, Kd_.data, Ki_.data);
                for(int i=0; i < n_joints_; i++)
                {
                    printf("Joint ID:%d \t", i+1);
                    printf("Kp;%0.3lf, Kd:%0.3lf ", Kp_.data(i), Kd_.data(i));
                    printf("q: %0.3lf, ", q_.data(i) * R2D);
                    printf("dq: %0.3lf, ", qd_.data(i) * R2D);
                    printf("qdot: %0.3lf, ", qdot_.data(i) * R2D);
                    printf("dqdot: %0.3lf, ", qd_dot_.data(i) * R2D);
                    printf("tau: %0.3f, %0.3f", torque(i), G(i));
                    printf("\n");
                }

                printf("\nForward Kinematics:\n");
                for(int j=0; j<NumChain; j++)
                {
                    printf("no.%d, PoE: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           ForwardPos[j](0), ForwardPos[j](1),ForwardPos[j](2),
                           ForwardOri[j](0), ForwardOri[j](1), ForwardOri[j](2));
                    double a, b, g;
                    x_[j].M.GetEulerZYX(a, b, g);
                    printf("no.%d, DH: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n",
                           j, x_[j].p(0), x_[j].p(1),x_[j].p(2), g, b, a);
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n",
                           j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);

                    printf("Inverse Condition Number: %0.5lf \n\n", InverseConditionNumber[j]);
                }
                printf("Dual-arm Manipulability Measure: %0.5lf\n\n", DAManipulabilityMeasure);
                printf("Right e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(0)*RADtoDEG, ex_(1)*RADtoDEG, ex_(2)*RADtoDEG, ex_(3), ex_(4), ex_(5));
                printf("Left e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",
                       ex_(6)*RADtoDEG, ex_(7)*RADtoDEG, ex_(8)*RADtoDEG, ex_(9), ex_(10), ex_(11));
                printf("\n*********************************************************\n");
                count = 0;

                //std::cout << "q0dot:" << std::endl;
                //std::cout << q0dot << "\n" << std::endl;

                //std::cout << "\n" << J1_kdl_.data << "\n"<< std::endl;
                //std::cout << J2_kdl_.data << "\n"<< std::endl;
                //std::cout << AJac << "\n"<< std::endl;
                //std::cout << spaceJac<< "\n"<< std::endl;
                //std::cout << bodyJac << "\n"<< std::endl;

            }
            count++;
        }

    private:
        // others
        double t=0.0;
        int ctr_obj_=0;
        int ik_mode_=0;
        double InitTime=0.0;

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
        KDL::Jacobian J1_inv_kdl_, J2_inv_kdl_;

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
        double DAManipulabilityMeasure;

        // kdl and Eigen Jacobian
        Eigen::MatrixXd spaceJac, bodyJac;
        Eigen::MatrixXd pInvJac;
        Eigen::MatrixXd AJac;
        Eigen::MatrixXd ScaledTransJac;

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_, q1dot_, q2dot_;
        Eigen::VectorXd q0dot;
        Eigen::VectorXd torque;

        // Task Space State
        // ver. 01
        KDL::Frame xd_[2]; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_[2];
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::VectorXd ex_;
        Eigen::VectorXd ex_dot_;
        Eigen::VectorXd dx;
        Eigen::VectorXd dxdot;

        // Input
        KDL::JntArray x_cmd_;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_, K_inf_;
        double K_trans, K_rot;
        Eigen::VectorXd CLIK_GAIN;

        // ros subscriber
        ros::Subscriber sub_x_cmd_;

        // ros publisher

        std::shared_ptr<SerialManipulator> cManipulator;
        std::unique_ptr<HYUControl::Controller> Control;
    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::ClosedLoopIK_Control,controller_interface::ControllerBase)