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
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics

#include <kdl/treefksolver.hpp>
#include <kdl/treeidsolver.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>



#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>
#include <Controller.h>

#define _USE_MATH_DEFINES

#include "SerialManipulator.h"

#define D2R M_PI/180.0
#define R2D 180.0/M_PI
#define num_taskspace 6
#define SaveDataMax 97

#define A 0.05
#define b 0.364
#define f 0.5

namespace  dualarm_controller
{
    class ComputedTorque_Control_CLIK : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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

            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (n.getParam("/dualarm/computedtorque_control_clik/gains/dualarm_joint" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/dualarm/computedtorque_control_clik/gains/dualarm_joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/dualarm/computedtorque_control_clik/gains/dualarm_joint" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/dualarm/computedtorque_control_clik/gains/dualarm_joint" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }
            }

            // 1.2.2 Closed-loop Inverse Kinematics Controller
            if (ctr_obj_ == 1)
            {
                if (!n.getParam("/dualarm/computedtorque_control_clik/clik_gain/K_regulation", K_regulation_))
                {
                    ROS_ERROR("Cannot find clik regulation gain");
                    return false;
                }
            }

            else if (ctr_obj_ == 2)
            {
                if (!n.getParam("/dualarm/computedtorque_control_clik/clik_gain/K_tracking", K_tracking_))
                {
                    ROS_ERROR("Cannot find clik tracking gain");
                    return false;
                }
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

            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            jnt_to_jac_solver_2.reset(new KDL::ChainJntToJacSolver(kdl_chain2_));
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            fk_pos_solver_2.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain2_));


            // ********* 5. 각종 변수 초기화 *********

            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(2*num_taskspace);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_chain[0].data = Eigen::VectorXd::Zero(8);
            q_chain[1].data = Eigen::VectorXd::Zero(8);


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
                    1, &ComputedTorque_Control_CLIK::commandCB,
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
            ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");

            cManipulator = new SerialManipulator;
            Control = new HYUControl::Controller(cManipulator);

            cManipulator->UpdateManipulatorParam();

            Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data);
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
                if(i >= 0 && i <= 7 )
                {
                    q_chain[0](i) = joints_[i].getPosition();
                }
            }

            Map<VectorXd>(q, 14) = q_.data;
            Map<VectorXd>(qdot, 14) = qdot_.data;

            cManipulator->pKin->PrepareJacobian(q);
            cManipulator->pDyn->PrepareDynamics(q, qdot);

            cManipulator->pKin->GetpinvJacobian(pInvJac);
            cManipulator->pKin->GetAnalyticJacobian(AJac);
            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);

            q_chain[0](0) = q_(0);
            q_chain[0](1) = q_(1);
            q_chain[0](2) = q_(2);
            q_chain[0](3) = q_(3);
            q_chain[0](4) = q_(4);
            q_chain[0](5) = q_(5);
            q_chain[0](6) = q_(6);
            q_chain[0](7) = q_(7);

            q_chain[1](0) = q_(0);
            q_chain[1](1) = q_(1);
            q_chain[1](2) = q_(8);
            q_chain[1](3) = q_(9);
            q_chain[1](4) = q_(10);
            q_chain[1](5) = q_(11);
            q_chain[1](6) = q_(12);
            q_chain[1](7) = q_(13);

            jnt_to_jac_solver_->JntToJac(q_chain[0], J_[0]);
            jnt_to_jac_solver_2->JntToJac(q_chain[1], J_[1]);

            fk_pos_solver_->JntToCart(q_chain[0], x_[0]);
            fk_pos_solver_2->JntToCart(q_chain[1], x_[1]);

            xd_dot_.setZero();

            if(t <= 5.0)
            {
                clik_flag = 0;


                //qd_.data.setZero();

                qd_.data(0) = 0;
                qd_.data(1) = 15.0*D2R;
                qd_.data(2) = 10.0*D2R;
                qd_.data(3) = -40.0*D2R;
                qd_.data(4) = 0;
                qd_.data(5) = 0;
                qd_.data(6) = -50.0*D2R;
                qd_.data(7) = 0;
                qd_.data(8) = -qd_.data(2);
                qd_.data(9) = qd_.data(3);
                qd_.data(10) = qd_.data(4);
                qd_.data(11) = qd_.data(5);
                qd_.data(12) = qd_.data(6);
                qd_.data(13) = qd_.data(7);

                qd_old_ = qd_;


            } else{
                if (ctr_obj_ == 1)
                {
                    xd_[0].p(0) = 0.364;
                    xd_[0].p(1) = -0.204;
                    xd_[0].p(2) = 0.326;
                    xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(-0.14, -1.13, 0.06));

                    xd_[1].p(0) = 0.364;
                    xd_[1].p(1) = 0.204;
                    xd_[1].p(2) = 0.326;
                    xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(-0.18, -1.10, -0.15));

                    for(int i=0; i<2; i++)
                    {
                        ex_temp_ = diff(x_[i], xd_[i]);


                        ex_.setZero();
                        ex_(0) = ex_temp_(0);
                        ex_(1) = ex_temp_(1);
                        ex_(2) = ex_temp_(2);
                        ex_(3) = ex_temp_(3);
                        ex_(4) = ex_temp_(4);
                        ex_(5) = ex_temp_(5);
                    }


                    qd_.data = qd_old_.data + pInvJac * (xd_dot_ ) * dt;
                    qd_old_.data = qd_.data;

                }
                else if (ctr_obj_ == 2)
                {




                    if (ik_mode_ == 1) // Open-loop Inverse Kinematics
                    {

                        /*
                        ex_(0) = ForwardOri[0](0);
                        ex_(1) = ForwardOri[0](1);
                        ex_(2) = ForwardOri[0](2);
                        ex_(3) = ForwardPos[0](0);
                        ex_(4) = ForwardPos[0](1);
                        ex_(5) = ForwardPos[0](2);

                        ex_(6) = ForwardOri[1](0);
                        ex_(7) = ForwardOri[1](1);
                        ex_(8) = ForwardOri[1](2);
                        ex_(9) = ForwardPos[1](0);
                        ex_(10) = ForwardPos[1](1);
                        ex_(11) = ForwardPos[1](2);
                         */

                        ex_(0) = -0.14;
                        ex_(1) = -1.13;
                        ex_(2) = 0.06;
                        ex_(3) = 0.364;
                        ex_(4) = -0.204;
                        ex_(5) = 0.326;

                        ex_(6) = -0.18;
                        ex_(7) = -1.1;
                        ex_(8) = -0.15;
                        ex_(9) =0.364;
                        ex_(10) = 0.204;
                        ex_(11) = 0.326;

                        xd_dot_(0) = 0;
                        xd_dot_(1) = 0.01;
                        xd_dot_(2) = 0;
                        xd_dot_(3) = 0;
                        xd_dot_(4) = 0;
                        xd_dot_(5) = 0;
                        xd_dot_(6) = 0;
                        xd_dot_(7) = 0;
                        xd_dot_(8) = 0;
                        xd_dot_(9) = 0;
                        xd_dot_(10) = 0;
                        xd_dot_(11) = 0;

                        qd_.data = qd_old_.data + pInvJac * (xd_dot_) * dt;
                        qd_old_.data = qd_.data;

                        //std::cout << qd_.data << std::endl;
                        //ROS_ERROR_STREAM("STOP!");
                        //ROS_ERROR("Could not find root link name");
                    }
                    else if (ik_mode_ == 2) // Closed-loop Inverse Kinematics
                    {
                        xd_[0].p(0) = A * sin(f * M_PI * (t - 10)) + b;
                        xd_[0].p(1) = -0.204;
                        xd_[0].p(2) = 0.326;
                        xd_[0].M = KDL::Rotation(KDL::Rotation::RPY(-0.14, -1.13, 0.06));

                        xd_dot_(0) = 0;
                        xd_dot_(1) = 0;
                        xd_dot_(2) = 0;
                        xd_dot_(3) = (f * M_PI) * A * cos(f * M_PI * (t - 10));
                        xd_dot_(4) = 0;
                        xd_dot_(5) = 0;

                        xd_[1].p(0) = 0.364;
                        xd_[1].p(1) = 0.204;
                        xd_[1].p(2) = 0.326;
                        xd_[1].M = KDL::Rotation(KDL::Rotation::RPY(-0.18, -1.10, -0.15));

                        for(int i=0; i<2; i++)
                        {
                            ex_temp_ = diff(x_[i], xd_[i]);
                            ex_.setZero();
                            ex_(0) = ex_temp_(3);
                            ex_(1) = ex_temp_(4);
                            ex_(2) = ex_temp_(5);
                            ex_(3) = ex_temp_(0);
                            ex_(4) = ex_temp_(1);
                            ex_(5) = ex_temp_(2);
                        }

                        qd_.data = qd_old_.data + pInvJac * (xd_dot_ + K_tracking_ * ex_) * dt;
                        qd_old_.data = qd_.data;



                    }

                }
            }

            Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, torque, dt);

            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque[i]);
                //joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            save_data();

            // ********* 5. state 출력 *********
            print_state();
        }

        void stopping(const ros::Time &time) override
        {
            delete Control;
            delete cManipulator;
        }

        static void save_data()
        {

        }

        void print_state()
        {
            static int count = 0;
            if (count > 99)
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
                    printf("Kp;%0.3lf, Kd:%0.3lf, Kinf:%0.3lf, ", Kp_.data(i), Kd_.data(i), Ki_.data(i));
                    printf("q: %0.3lf, ", q_.data(i) * 1);
                    printf("dq: %0.3lf, ", qd_.data(i) * 1);
                    printf("qdot: %0.3lf, ", qdot_.data(i) * 1);
                    printf("dqdot: %0.3lf, ", qd_dot_.data(i) * 1);
                    printf("tau: %0.3f", torque[i]);
                    printf("\n");
                }
                printf("Forward Kinematics:\n");
                for(int j=0; j<NumChain; j++)
                {
                    printf("x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.2lf, v:%0.2lf, w:%0.2lf\n",
                           ForwardPos[j](0), ForwardPos[j](1),ForwardPos[j](2), ForwardOri[j](0), ForwardOri[j](1), ForwardOri[j](2));

                }
                printf("R eu:%0.3lf, ev:%0.3lf, ew:%0.3lf, ex:%0.2lf, ey:%0.2lf, ez:%0.2lf\n",ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5));
                printf("L eu:%0.3lf, ev:%0.3lf, ew:%0.3lf, ex:%0.2lf, ey:%0.2lf, ez:%0.2lf\n",ex_(6), ex_(7), ex_(8), ex_(9), ex_(10), ex_(11));
                //std::cout << "ForwardKin SE3\n" << cManipulator->pKin->GetForwardKinematicsSE3(NumChain) << std::endl;

                //std::cout << AJac << std::endl;
                count = 0;
            }
            count++;
        }

    private:
        // others
        double t;
        int ctr_obj_;
        int ik_mode_;
        int event;
        int clik_flag;

        //Joint handles
        unsigned int n_joints_;                               // joint 숫자
        std::vector<std::string> joint_names_;                // joint name ??
        std::vector<hardware_interface::JointHandle> joints_; // ??
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

        // kdl
        KDL::Tree kdl_tree_;   // tree?
        KDL::Chain kdl_chain_; // chain?
        KDL::Chain kdl_chain2_;

        Vector3d ForwardPos[2];
        Vector3d ForwardOri[2];
        int NumChain=2;

        // kdl and Eigen Jacobian
        MatrixXd pInvJac;
        KDL::Jacobian J_[2];
        //KDL::Jacobian J_inv_;
        //Eigen::Matrix<double, num_taskspace, num_taskspace> J_inv_;
        Eigen::MatrixXd J_chain;
        Eigen::MatrixXd J_inv_;
        Eigen::MatrixXd AJac;
        Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_2;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_2;


        // Joint Space State
        KDL::JntArray qd_, qd_dot_, qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, qdot_;

        KDL::JntArray qd_clik[2];
        KDL::JntArray qd_dot_clik[2];

        KDL::JntArray q_chain[2];
        KDL::JntArray qd_chain[2];
        KDL::JntArray qd_chain_old[2];

        double q[14], qdot[14];
        double dq[14] = {0.0,};
        double dqdot[14] = {0.0,};
        double dqddot[14] = {0.0,};
        double torque[14] = {0.0,};

        // Task Space State
        // ver. 01
        KDL::Frame xd_[2]; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_[2];
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::Matrix<double, 2*num_taskspace, 1> ex_;
        Eigen::Matrix<double, 2*num_taskspace, 1> xd_dot_, xd_ddot_[2];
        Eigen::Matrix<double, 2*num_taskspace, 1> xdot_;
        Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_;

        // Input
        KDL::JntArray x_cmd_;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_;
        double K_regulation_, K_tracking_;

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
        std_msgs::Float64MultiArray msg_xd_, msg_x_, msg_ex_;
        std_msgs::Float64MultiArray msg_SaveData_;

        SerialManipulator *cManipulator;
        HYUControl::Controller *Control;


    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::ComputedTorque_Control_CLIK,controller_interface::ControllerBase)