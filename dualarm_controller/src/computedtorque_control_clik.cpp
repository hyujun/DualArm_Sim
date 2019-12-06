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
// #include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>
#define _USE_MATH_DEFINES
#include <Eigen/Dense>

#include "SerialManipulator.h"

#define R2D M_PI/180.0
#define D2R 180.0/M_PI
#define num_taskspace 6
#define SaveDataMax 97
#define A 0.1
#define b 2.5
#define f 1
#define t_set 1

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

            q_.resize(n_joints_);
            qdot_.resize(n_joints_);
            torque_.resize(n_joints_);


            qd_.resize(n_joints_);
            qd_dot_.resize(n_joints_);
            qd_ddot_.resize(n_joints_);



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
            sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command",
                    1, &ComputedTorque_Control_CLIK::commandCB,
                    this);
            event = 0; // subscribe 받기 전: 0
            // subscribe 받은 후: 1

            return true;
        }

        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if (msg->data.size() != num_taskspace)
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
                return;
            }

            for (int i = 0; i < num_taskspace; i++)
            {
                x_cmd_(i) = msg->data[i];
            }

            event = 1; // subscribe 받기 전: 0
            // subscribe 받은 후: 1
        }

        void starting(const ros::Time &time)
        {
            t = 0.0;
            ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
        }

        void update(const ros::Time &time, const ros::Duration &period)
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
                torque_(i) = joints_[i].getEffort();
            }

            //Implement the control code



            for (int i = 0; i < n_joints_; i++)
            {
                //joints_[i].setCommand(tau_d_(i));
                joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            save_data();

            // ********* 5. state 출력 *********
            print_state();
        }

        void stopping(const ros::Time &time)
        {
        }

        void save_data()
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
                for(int i=0; i < n_joints_; i++)
                {
                    printf("Joint ID:%d", i);
                    printf("q: %f, ", q_(0) * R2D);
                    printf("dq: %f, ", q_(1) * R2D);
                    printf("qdot: %f, ", q_(2) * R2D);
                    printf("dqdot: %f, ", q_(3) * R2D);
                    printf("\n");
                }


                //printf("*** Desired Position in Task Space (unit: m) ***\n");
                //printf("xd: %f, ", xd_.p(0));
                //printf("yd: %f, ", xd_.p(1));
                //printf("zd: %f\n", xd_.p(2));
                //printf("\n");

                //printf("*** Actual Position in Task Space (unit: m) ***\n");
                //printf("x: %f, ", x_.p(0));
                //printf("y: %f, ", x_.p(1));
                //printf("z: %f\n", x_.p(2));
                //printf("\n");

                // printf("*** Desired Orientation in Task Space (unit: ??) ***\n");
                // printf("xd_(0): %f, ", xd_.M(KDL::Rotation::RPY(0));
                // printf("xd_(1): %f, ", xd_.M(KDL::Rotation::RPY(1));
                // printf("xd_(2): %f\n", xd_.M(KDL::Rotation::RPY(2));
                // printf("\n");

                // printf("*** Actual Orientation in Task Space (unit: ??) ***\n");
                // printf("x_(0): %f, ", x_.M(KDL::Rotation::RPY(0));
                // printf("x_(1): %f, ", x_.M(KDL::Rotation::RPY(1));
                // printf("x_(2): %f\n", x_.M(KDL::Rotation::RPY(2));
                // printf("\n");
                /*
                printf("*** Desired Translation Velocity in Task Space (unit: m/s) ***\n");
                printf("xd_dot: %f, ", xd_dot_(0));
                printf("yd_dot: %f, ", xd_dot_(1));
                printf("zd_dot: %f\n", xd_dot_(2));
                printf("\n");

                printf("*** Actual Translation Velocity in Task Space (unit: m/s) ***\n");
                printf("xdot: %f, ", xdot_(0));
                printf("ydot: %f  ", xdot_(1));
                printf("zdot: %f\n", xdot_(2));
                printf("\n");

                printf("*** Desired Angular Velocity in Task Space (unit: rad/s) ***\n");
                printf("rd_dot: %f, ", xd_dot_(3));
                printf("pd_dot: %f, ", xd_dot_(4));
                printf("yd_dot: %f\n", xd_dot_(5));
                printf("\n");

                printf("*** Actual Angular Velocity in Task Space (unit: rad/s) ***\n");
                printf("r_dot: %f, ", xdot_(3));
                printf("p_dot: %f  ", xdot_(4));
                printf("y_dot: %f\n", xdot_(5));
                printf("\n");

                printf("*** Desired Rotation Matrix of end-effector ***\n");
                printf("%f, ",xd_.M(0,0));
                printf("%f, ",xd_.M(0,1));
                printf("%f\n",xd_.M(0,2));
                printf("%f, ",xd_.M(1,0));
                printf("%f, ",xd_.M(1,1));
                printf("%f\n",xd_.M(1,2));
                printf("%f, ",xd_.M(2,0));
                printf("%f, ",xd_.M(2,1));
                printf("%f\n",xd_.M(2,2));
                printf("\n");

                printf("*** Actual Rotation Matrix of end-effector ***\n");
                printf("%f, ",x_.M(0,0));
                printf("%f, ",x_.M(0,1));
                printf("%f\n",x_.M(0,2));
                printf("%f, ",x_.M(1,0));
                printf("%f, ",x_.M(1,1));
                printf("%f\n",x_.M(1,2));
                printf("%f, ",x_.M(2,0));
                printf("%f, ",x_.M(2,1));
                printf("%f\n",x_.M(2,2));
                printf("\n");

                printf("*** Joint Space Error (unit: deg)  ***\n");
                printf("q1: %f, ", R2D * e_(0));
                printf("q2: %f, ", R2D * e_(1));
                printf("q3: %f, ", R2D * e_(2));
                printf("q4: %f, ", R2D * e_(3));
                printf("q5: %f, ", R2D * e_(4));
                printf("q6: %f\n", R2D * e_(5));
                printf("\n");

                printf("*** Task Space Position Error (unit: mm) ***\n");
                printf("x: %f, ", ex_(0)*1000);
                printf("y: %f, ", ex_(1)*1000);
                printf("z: %f\n", ex_(2)*1000);
                printf("\n");

                printf("*** Task Space Orientation Error ?? (unit: deg) ***\n");
                printf("r: %f, ", ex_(3)*R2D);
                printf("p: %f, ", ex_(4)*R2D);
                printf("y: %f\n", ex_(5)*R2D);
                printf("\n");
                */
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

        //Joint handles
        unsigned int n_joints_;                               // joint 숫자
        std::vector<std::string> joint_names_;                // joint name ??
        std::vector<hardware_interface::JointHandle> joints_; // ??
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

        // kdl
        KDL::Tree kdl_tree_;   // tree?
        KDL::Chain kdl_chain_; // chain?

        // kdl M,C,G
        KDL::JntSpaceInertiaMatrix M_; // intertia matrix
        KDL::JntArray C_;              // coriolis
        KDL::JntArray G_;              // gravity torque vector
        KDL::Vector gravity_;

        // kdl and Eigen Jacobian
        KDL::Jacobian J_;
        // KDL::Jacobian J_inv_;
        // Eigen::Matrix<double, num_taskspace, num_taskspace> J_inv_;
        Eigen::MatrixXd J_inv_;
        Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
        // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics

        // Joint Space State
        Eigen::VectorXd q_, qdot_;
        Eigen::VectorXd qd_, qd_dot_, qd_ddot_;
        Eigen::VectorXd qd_old_;
        Eigen::VectorXd torque_;
        Eigen::VectorXd e_, e_dot_, e_int_;

        // Task Space State
        // ver. 01
        KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_;
        KDL::Twist ex_temp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::Matrix<double, num_taskspace, 1> ex_;
        Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_;
        Eigen::Matrix<double, num_taskspace, 1> xdot_;
        Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_;

        // ver. 02
        // Eigen::Matrix<double, num_taskspace, 1> xd_, xd_dot_, xd_ddot_;
        // Eigen::Matrix<double, num_taskspace, 1> x_, xdot_;
        // KDL::Frame x_temp_;
        // Eigen::Matrix<double, num_taskspace, 1> ex_, ex_dot_, ex_int_;

        // Input
        KDL::JntArray x_cmd_;

        // Torque
        KDL::JntArray aux_d_;
        KDL::JntArray comp_d_;
        KDL::JntArray tau_d_;

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

    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::ComputedTorque_Control_CLIK,controller_interface::ControllerBase)