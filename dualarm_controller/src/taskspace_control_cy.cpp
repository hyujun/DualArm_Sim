//
// Created by june on 19. 12. 5..
//

// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

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

#define A 0.25
#define b1 0.451
#define b2 -0.316
#define b3 0.843
#define f 0.3

#define l_p1 0.470
#define l_p2 0.40
#define l_p3 0.812

#define Deg_A 70
#define Deg_f 0.5

namespace dualarm_controller
{
    class Taskspace_Control_CY : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
            Kp_.resize(n_joints_);
            Kd_.resize(n_joints_);
            Ki_.resize(n_joints_);
            K_inf_.resize(n_joints_);
            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_), K_inf(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (n.getParam("/dualarm/taskspace_control_cy/gains/dualarm_joint" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/dualarm/taskspace_control_cy/gains/dualarm_joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/dualarm/taskspace_control_cy/gains/dualarm_joint" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/dualarm/taskspace_control_cy/gains/dualarm_joint" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }

                if (n.getParam("/dualarm/taskspace_control_cy/gains/dualarm_joint" + si + "/pid/h", K_inf[i]))
                {
                    K_inf_(i) = K_inf[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/h gain");
                    return false;
                }
            }

            // 1.2.2 Closed-loop Inverse Kinematics Controller

            if (!n.getParam("/dualarm/taskspace_control_cy/clik_gain/K_pos", K_regulation_))
            {
                ROS_ERROR("Cannot find clik regulation gain");
                return false;
            }

            if (!n.getParam("/dualarm/taskspace_control_cy/clik_gain/K_ori", K_tracking_))
            {
                ROS_ERROR("Cannot find clik tracking gain");
                return false;
            }

            CLIK_GAIN.setZero(6*2, 6*2);
            CLIK_GAIN(0,0) = K_tracking_;
            CLIK_GAIN(1,1) = K_tracking_;
            CLIK_GAIN(2,2) = K_tracking_;
            CLIK_GAIN(3,3) = K_regulation_;
            CLIK_GAIN(4,4) = K_regulation_;
            CLIK_GAIN(5,5) = K_regulation_;
            CLIK_GAIN(6,6) = K_tracking_;
            CLIK_GAIN(7,7) = K_tracking_;
            CLIK_GAIN(8,8) = K_tracking_;
            CLIK_GAIN(9,9) = K_regulation_;
            CLIK_GAIN(10,10) = K_regulation_;
            CLIK_GAIN(11,11) = K_regulation_;

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
            x_cmd_.data = Eigen::VectorXd::Zero(2*num_taskspace);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
            pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
            pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

            pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
            pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
            pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

            pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

            // 6.2 subsriber
            sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>(
                    "command",
                    1, &Taskspace_Control_CY::commandCB,
                    this);
            event = 0; // subscribe 받기 전: 0
            // subscribe 받은 후: 1

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

            event = 1; // subscribe 받기 전: 0
            // subscribe 받은 후: 1
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            ROS_INFO("Starting Task space Controller");

            cManipulator = new SerialManipulator;
            Control = new HYUControl::Controller(cManipulator);

            cManipulator->UpdateManipulatorParam();

            Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data, K_inf_.data);

            dx.setZero();
            dxdot.setZero();
        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();
            t = t + 0.001;

            // 0.2 joint state
            for (int i = 0; i < n_joints_; i++)
            {
                q_(i) = joints_[i].getPosition();
                qdot_(i) = joints_[i].getVelocity();
                //torque_(i) = joints_[i].getEffort();
            }

            Map<VectorXd>(q, n_joints_) = q_.data;
            Map<VectorXd>(qdot, n_joints_) = qdot_.data;

            cManipulator->pKin->PrepareJacobian(q);
            cManipulator->pDyn->PrepareDynamics(q, qdot);

            cManipulator->pDyn->MG_Mat_Joint(M, G);

            cManipulator->pKin->GetSpaceJacobian(spaceJac);
            cManipulator->pKin->GetBodyJacobian(bodyJac);
            cManipulator->pKin->GetpinvJacobian(pInvJac);
            cManipulator->pKin->GetAnalyticJacobian(AJac);
            cManipulator->pKin->GetScaledTransJacobian(ScaledTransJac);
            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);
            cManipulator->pKin->GetAngleAxis(ForwardAxis, ForwardAngle, NumChain);

            xd_dot_.setZero();
            ex_.setZero(12);
            ex1_.setZero(12);
            ex_dot_.setZero(12);
            
            InitTime=5.0;

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
            else{
                if (ctr_obj_ == 1)
                {
                    xd_[0].p(0) = b1;
                    xd_[0].p(1) = b2;
                    xd_[0].p(2) = b3;
                    xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(M_PI/2, 0, 0));

                    xd_dot_.setZero();

                    xd_[1].p(0) = l_p1;
                    xd_[1].p(1) = l_p2;
                    xd_[1].p(2) = l_p3;
                    xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(0, M_PI/2, 0));

                    ex_.setZero();
                    ex_temp_ = diff(xd_[0], x_[0]);
                    ex_(0) = ex_temp_(3);
                    ex_(1) = ex_temp_(4);
                    ex_(2) = ex_temp_(5);
                    ex_(3) = ex_temp_(0);
                    ex_(4) = ex_temp_(1);
                    ex_(5) = ex_temp_(2);

                    ex_temp_ = diff(xd_[1], x_[1]);
                    ex_(6) = ex_temp_(3);
                    ex_(7) = ex_temp_(4);
                    ex_(8) = ex_temp_(5);
                    ex_(9) = ex_temp_(0);
                    ex_(10) = ex_temp_(1);
                    ex_(11) = ex_temp_(2);
                }
                else if (ctr_obj_ == 2)
                {
                    if (ik_mode_ == 1)
                    {
                        xd_[0].p(0) =
                        xd_[0].p(1) = b2;
                        xd_[0].p(2) = b3;
                        xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(0, -M_PI/2, 0));

                        xd_dot_.setZero();
                        xd_dot_(2+1) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                        xd_[1].p(0) = l_p1;
                        xd_[1].p(1) = l_p2;
                        xd_[1].p(2) = l_p3;
                        xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(M_PI/2, 0 , 0));

                        x_[0].p(0) = ForwardPos[0](0);
                        x_[0].p(1) = ForwardPos[0](1);
                        x_[0].p(2) = ForwardPos[0](2);
                        x_[0].M = KDL::Rotation(KDL::Rotation::RPY(ForwardOri[0](0), ForwardOri[0](1), ForwardOri[0](2)));

                        x_[1].p(0) = ForwardPos[1](0);
                        x_[1].p(1) = ForwardPos[1](1);
                        x_[1].p(2) = ForwardPos[1](2);
                        x_[1].M = KDL::Rotation(KDL::Rotation::RPY(ForwardOri[1](0), ForwardOri[1](1), ForwardOri[1](2)));

                        ex_.setZero();
                        ex_temp_ = diff(xd_[0], x_[0]);
                        ex_(0) = ex_temp_(3);
                        ex_(1) = ex_temp_(4);
                        ex_(2) = ex_temp_(5);
                        ex_(3) = ex_temp_(0);
                        ex_(4) = ex_temp_(1);
                        ex_(5) = ex_temp_(2);

                        ex_temp_ = diff(xd_[1], x_[1]);
                        ex_(6) = ex_temp_(3);
                        ex_(7) = ex_temp_(4);
                        ex_(8) = ex_temp_(5);
                        ex_(9) = ex_temp_(0);
                        ex_(10) = ex_temp_(1);
                        ex_(11) = ex_temp_(2);
                        ex_.tail(6).setZero();
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
                        dx(8) = 0; //M_PI/2;
                        dx(9) = l_p1;
                        dx(10) = l_p2;
                        dx(11) = l_p3;

                        xd_dot_.setZero();
                        xd_dot_(4) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                        //xd_dot_(11) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                        dSE3.block<3,1>(0,3) = dx.segment(3,3);
                        cManipulator->pKin->RollPitchYawtoSO3(dx(0), dx(1), dx(2), dSO3);
                        dSE3.block<3,3>(0,0) = dSO3;
                        EndNum=9;
                        aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                        eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                        cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                        ex_(0) = eAxis(0);
                        ex_(1) = eAxis(1);
                        ex_(2) = eAxis(2);
                        ex_(3) = eSE3(0,3);
                        ex_(4) = eSE3(1,3);
                        ex_(5) = eSE3(2,3);

                        dSE3.block<3,1>(0,3) = dx.segment(9,3);
                        cManipulator->pKin->RollPitchYawtoSO3(dx(6), dx(7), dx(8), dSO3);
                        dSE3.block<3,3>(0,0) = dSO3;
                        EndNum=16;
                        aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                        eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                        cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                        ex_(6) = eAxis(0);
                        ex_(7) = eAxis(1);
                        ex_(8) = eAxis(2);
                        ex_(9) = eSE3(0,3);
                        ex_(10) = eSE3(1,3);
                        ex_(11) = eSE3(2,3);
                    }
                    else if (ik_mode_ == 3)
                    {
                        dx(0) = 0;
                        dx(1) = -M_PI_2;
                        dx(2) = 0;
                        dx(3) = b1;
                        dx(4) = b2;
                        dx(5) = A * sin(f * M_PI * (t - InitTime)) + b3;

                        dx(6) = 0;
                        dx(7) = -M_PI_2;
                        dx(8) = 0; //M_PI/2;
                        dx(9) = l_p1;
                        dx(10) = l_p2;
                        dx(11) = l_p3;

                        xd_dot_.setZero();
                        xd_dot_(5) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                        //xd_dot_(11) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                        dSE3.block<3,1>(0,3) = dx.segment(3,3);
                        cManipulator->pKin->RollPitchYawtoSO3(dx(0), dx(1), dx(2), dSO3);
                        dSE3.block<3,3>(0,0) = dSO3;
                        EndNum=9;
                        aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                        eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                        cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                        ex_(0) = eAxis(0);
                        ex_(1) = eAxis(1);
                        ex_(2) = eAxis(2);
                        ex_(3) = eSE3(0,3);
                        ex_(4) = eSE3(1,3);
                        ex_(5) = eSE3(2,3);

                        dSE3.block<3,1>(0,3) = dx.segment(9,3);
                        cManipulator->pKin->RollPitchYawtoSO3(dx(6), dx(7), dx(8), dSO3);
                        dSE3.block<3,3>(0,0) = dSO3;
                        EndNum=16;
                        aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                        eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                        cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                        ex_(6) = eAxis(0);
                        ex_(7) = eAxis(1);
                        ex_(8) = eAxis(2);
                        ex_(9) = eSE3(0,3);
                        ex_(10) = eSE3(1,3);
                        ex_(11) = eSE3(2,3);
                        //ex_.tail(6).setZero();
                    }
                }
                else if( ctr_obj_ == 3 )
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

                    xd_dot_.setZero();
                    //xd_dot_(5) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));
                    //xd_dot_(11) = (f * M_PI) * A * cos(f * M_PI * (t - InitTime));

                    dSE3.block<3,1>(0,3) = dx.segment(3,3);
                    cManipulator->pKin->RollPitchYawtoSO3(dx(0), dx(1), dx(2), dSO3);
                    dSE3.block<3,3>(0,0) = dSO3;
                    EndNum=9;
                    aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                    eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                    cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                    ex_(0) = eAxis(0);
                    ex_(1) = eAxis(1);
                    ex_(2) = eAxis(2);
                    ex_(3) = eSE3(0,3);
                    ex_(4) = eSE3(1,3);
                    ex_(5) = eSE3(2,3);

                    dSE3.block<3,1>(0,3) = dx.segment(9,3);
                    cManipulator->pKin->RollPitchYawtoSO3(dx(6), dx(7), dx(8), dSO3);
                    dSE3.block<3,3>(0,0) = dSO3;
                    EndNum=16;
                    aSE3 = cManipulator->pKin->GetForwardKinematicsSE3(EndNum);
                    eSE3 = cManipulator->pKin->inverse_SE3(dSE3)*aSE3;
                    cManipulator->pKin->SO3toRollPitchYaw(eSE3.block(0,0,3,3), eAxis);
                    ex_(6) = eAxis(0);
                    ex_(7) = eAxis(1);
                    ex_(8) = eAxis(2);
                    ex_(9) = eSE3(0,3);
                    ex_(10) = eSE3(1,3);
                    ex_(11) = eSE3(2,3);
                    //ex_.tail(6).setZero();
                }
            }

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

            if( ctr_obj_ == 3)
            {
                Control->TaskInvDynController(ex_, ex_dot_, q_.data, qdot_.data, torque, dt);
            }
            else
            {
                double alpha = 0.001;
                q0dot = alpha*(q_.data.cwiseInverse()*cManipulator->pKin->GetManipulabilityMeasure());
                //qd_dot_.data = pInvJac * (xd_dot_ - CLIK_GAIN * ex_) + (Eigen::MatrixXd::Identity(16,16) - AJac.transpose()*AJac)*q0dot;
                qd_dot_.data = AJac.transpose() * (xd_dot_ - CLIK_GAIN * ex_) + (Eigen::MatrixXd::Identity(16,16) - AJac.transpose()*AJac)*q0dot;
                //qd_dot_.data = ScaledTransJac * (xd_dot_ - CLIK_GAIN * ex_);

                qd_.data = qd_old_.data + qd_dot_.data * dt;
                qd_old_.data = qd_.data;
                Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);
            }

            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque[i]);
                //joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            publish_data();

            // ********* 5. state 출력 *********
            print_state();
        }

        void stopping(const ros::Time &time) override
        {
            delete Control;
            delete cManipulator;
        }

        void publish_data()
        {
            msg_q_.data.clear();
            msg_qd_.data.clear();
            msg_e_.data.clear();
            msg_qddot_.data.clear();
            msg_x_.data.clear();
            msg_xd_.data.clear();
            msg_ex_.data.clear();

            for(int i=0; i < n_joints_; i++)
            {
                msg_q_.data.push_back(q_(i));
                msg_qd_.data.push_back(qd_(i));
                msg_e_.data.push_back(qd_(i) - q_(i));
            }

            for(int j=0; j<2*num_taskspace; j++)
            {
                msg_xd_.data.push_back(dx(j));
                msg_ex_.data.push_back(ex_(j));
            }

            pub_q_.publish(msg_q_);
            pub_qd_.publish(msg_qd_);
            pub_e_.publish(msg_e_);

            pub_xd_.publish(msg_xd_);
            pub_ex_.publish(msg_ex_);
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
                if (event == 0)
                {
                    printf("No Active!!!\n");
                }
                else
                {
                    printf("Active!!!\n");
                }

                printf("*** States in Joint Space (unit: deg) ***\n");
                Control->GetPIDGain(Kp_.data, Kd_.data, Ki_.data);
                for(int i=0; i < n_joints_; i++)
                {
                    printf("Joint ID:%d \t", i+1);
                    //printf("Kp;%0.3lf, Kd:%0.3lf, Kinf:%0.3lf, ", Kp_.data(i), Kd_.data(i), Ki_.data(i));
                    printf("q: %0.3lf, ", q_.data(i) * R2D);
                    printf("dq: %0.3lf, ", qd_.data(i) * R2D);
                    printf("qdot: %0.3lf, ", qdot_.data(i) * R2D);
                    printf("dqdot: %0.3lf, ", qd_dot_.data(i) * R2D);
                    printf("tau: %0.3f, %0.3f", torque[i], G(i));
                    printf("\n");
                }

                printf("\nForward Kinematics:\n");
                for(int j=0; j<NumChain; j++)
                {
                    printf("no.%d, PoE: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j,
                           ForwardPos[j](0), ForwardPos[j](1),ForwardPos[j](2), ForwardOri[j](0), ForwardOri[j](1), ForwardOri[j](2));
                    double a, b, g;
                    x_[j].M.GetEulerZYX(a, b, g);
                    printf("no.%d, DH: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n", j, x_[j].p(0), x_[j].p(1),x_[j].p(2), g, b, a);
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n\n", j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);


                }
                printf("Right e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",ex_(0)*RADtoDEG, ex_(1)*RADtoDEG, ex_(2)*RADtoDEG, ex_(3), ex_(4), ex_(5));
                printf("Left e(u):%0.3lf, e(v):%0.3lf, e(w):%0.3lf, e(x):%0.3lf, e(y):%0.3lf, e(z):%0.3lf\n",ex_(6)*RADtoDEG, ex_(7)*RADtoDEG, ex_(8)*RADtoDEG, ex_(9), ex_(10), ex_(11));
                printf("\n*********************************************************\n");
                count = 0;

                //std::cout << "\n" << J1_kdl_.data << "\n"<< std::endl;
                //std::cout << J2_kdl_.data << "\n"<< std::endl;
                //std::cout << AJac << "\n"<< std::endl;
                //std::cout << spaceJac<< "\n"<< std::endl;
                //std::cout << bodyJac << "\n"<< std::endl;
                //usleep(100000);

                //std::cout << CLIK_GAIN*ex_ << "\n"<< std::endl;
                //std::cout << xd_dot_ << "\n"<< std::endl;
                //std::cout << q0dot << "\n"<< std::endl;
                //usleep(100000);

                //std::cout << "\n" << M_kdl_.data << "\n"<< std::endl;
                //std::cout << M1_kdl_.data << "\n"<< std::endl;
                //std::cout << M << "\n"<< std::endl;
                //usleep(100000);
            }
            count++;
        }

    private:
        // others
        double t;
        int ctr_obj_;
        int ik_mode_;
        int event;
        double InitTime=0;

        SE3 dSE3, eSE3, aSE3;
        SO3 dSO3;
        Vector3d eAxis;
        double eAngle;
        int EndNum=0;

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

        KDL::Jacobian J1_kdl_, J2_kdl_;
        KDL::Jacobian J1_inv_kdl_, J2_inv_kdl_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_, fk_pos_solver1_; //Solver to compute the forward kinematics (position)
        //boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_, fk_vel_solver1_; //Solver to compute the forward kinematics (velocity)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_, jnt_to_jac_solver1_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_, id_solver1_;               // Solver To compute the inverse dynamics

        MatrixXd M;
        VectorXd G;

        Vector3d ForwardPos[2];
        Vector3d ForwardOri[2];
        int NumChain=2;
        Vector3d ForwardAxis[2];
        double ForwardAngle[2];

        // kdl and Eigen Jacobian
        Eigen::MatrixXd spaceJac, bodyJac;
        Eigen::MatrixXd pInvJac;
        Eigen::MatrixXd AJac;
        Eigen::MatrixXd ScaledTransJac;
        Eigen::MatrixXd BodyJac;

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_, q1dot_, q2dot_;
        Eigen::VectorXd q0dot;

        double q[16];
        double qdot[16];
        double torque[16];

        // Task Space State
        // ver. 01
        KDL::Frame xd_[2]; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_[2];
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::VectorXd ex_, ex1_;
        Eigen::VectorXd ex_dot_;
        Eigen::Matrix<double, 2*num_taskspace, 1> dx;
        Eigen::Matrix<double, 2*num_taskspace, 1> dxdot;
        Eigen::Matrix<double, 2*num_taskspace, 1> xd_dot_;

        // Input
        KDL::JntArray x_cmd_;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_, K_inf_;
        double K_regulation_, K_tracking_;
        Eigen::MatrixXd CLIK_GAIN;

        // save the data
        double SaveData_[SaveDataMax];

        // ros subscriber
        ros::Subscriber sub_x_cmd_;

        // ros publisher
        ros::Publisher pub_qd_, pub_q_, pub_e_;
        ros::Publisher pub_xd_, pub_x_, pub_ex_;
        ros::Publisher pub_SaveData_;

        // ros message
        std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
        std_msgs::Float64MultiArray msg_qddot_;
        std_msgs::Float64MultiArray msg_xd_, msg_x_, msg_ex_;
        std_msgs::Float64MultiArray msg_SaveData_;

        SerialManipulator *cManipulator;
        HYUControl::Controller *Control;


    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::Taskspace_Control_CY,controller_interface::ControllerBase)