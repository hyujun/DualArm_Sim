//
// Created by june on 19. 12. 5..
//

// from ros-control meta packages
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include "dualarm_controller/JointDesiredState.h"
#include "dualarm_controller/JointCurrentState.h"
#include "utils.h"

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
    class JointSpace_Control : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // ********* 1. Get joint name / gain from the parameter server *********

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
            aKp_.resize(n_joints_);
            aKd_.resize(n_joints_);
            aKi_.resize(n_joints_);
            aK_inf_.resize(n_joints_);
            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_), K_inf(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (n.getParam("/dualarm/jointspace_control/gains/dualarm_joint" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/dualarm/jointspace_control/gains/dualarm_joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/dualarm/jointspace_control/gains/dualarm_joint" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/dualarm/jointspace_control/gains/dualarm_joint" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }

                if (n.getParam("/dualarm/jointspace_control/gains/dualarm_joint" + si + "/pid/h", K_inf[i]))
                {
                    K_inf_(i) = K_inf[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/h gain");
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
            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            dq.setZero(n_joints_);

            des_torque.setZero(n_joints_);
            act_torque.setZero(n_joints_);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            state_pub_.reset(new realtime_tools::RealtimePublisher<dualarm_controller::JointCurrentState>(n, "states", 10));
            state_pub_->msg_.header.stamp = ros::Time::now();
            for(int i=0; i<(n_joints_-1); i++)
            {
                state_pub_->msg_.name.push_back(joint_names_[i].c_str()) ;
                state_pub_->msg_.q.push_back(q_.data(i));
                state_pub_->msg_.qdot.push_back(qdot_.data(i));
                state_pub_->msg_.dq.push_back(qd_.data(i));
                state_pub_->msg_.dqdot.push_back(qd_dot_.data(i));
                state_pub_->msg_.effort_command.push_back(act_torque(i));
                state_pub_->msg_.Kp.push_back(Kp_.data(i));
                state_pub_->msg_.Kd.push_back(Kd_.data(i));
            }
            pub_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            // 6.2 subsriber
            const auto joint_state_cb = utils::makeCallback<dualarm_controller::JointDesiredState>([&](const auto& msg){
                for(int i=0; i<n_joints_; i++)
                {
                    dq(i) = msg.dq[i].data*D2R;
                    Kp_.data(i) = msg.Kp[i].data;
                    Kd_.data(i) = msg.Kd[i].data;
                }
                qd_.data = dq;
            });
            sub_x_cmd_ = n.subscribe<dualarm_controller::JointDesiredState>( "command", 5, joint_state_cb);

            return true;
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            InitTime=5.0;

            ROS_INFO_STREAM("Starting Joint-Space Controller");

            cManipulator = std::make_shared<SerialManipulator>();
            Control = std::make_unique<HYUControl::Controller>(cManipulator);

            cManipulator->UpdateManipulatorParam();
        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            std::vector<double> &commands = *pub_buffer_.readFromRT();
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();

            // 0.2 joint state
            for (int i = 0; i < n_joints_; i++)
            {
                q_(i) = joints_[i].getPosition();
                qdot_(i) = joints_[i].getVelocity();
                act_torque(i) = joints_[i].getEffort();
            }

            //----------------------
            // dynamics calculation
            //----------------------
            cManipulator->pKin->PrepareJacobian(q_.data);
            cManipulator->pDyn->PrepareDynamics(q_.data, qdot_.data);
            cManipulator->pDyn->MG_Mat_Joint(M, G);
            //cManipulator->pDyn->C_Matrix(C);

            cManipulator->pKin->GetForwardKinematics(ForwardPos, ForwardOri, NumChain);
            cManipulator->pKin->GetAngleAxis(ForwardAxis, ForwardAngle, NumChain);
            Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data, K_inf_.data);
            Control->GetPIDGain(aKp_.data, aKd_.data, aKi_.data);

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

            if( t <= InitTime )
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
            }

            qd_ddot_.data.setZero();
            qd_dot_.data.setZero();

            Control->InvDynController(q_.data, qdot_.data, qd_.data, qd_dot_.data, qd_ddot_.data, des_torque, dt);

            for (int i = 0; i < n_joints_; i++)
            {
                if(des_torque(i) >= 300)
                    des_torque(i) = 300;
                else if(des_torque(i) <= -300)
                    des_torque(i) = -300;

                joints_[i].setCommand(des_torque(i));
            }

            // ********* 4. data 저장 *********
            publish_data();

            // ********* 5. state 출력 *********
            print_state();

            t = t + dt;
        }

        void stopping(const ros::Time &time) override
        {
            ROS_INFO_STREAM("Stop Joint-Space Controller");
        }

        void publish_data()
        {
            static int loop_count_ = 0;
            if(loop_count_ > 2)
            {
                if(state_pub_->trylock())
                {
                    state_pub_->msg_.header.stamp = ros::Time::now();
                    for(size_t i=0; i<(n_joints_-1); i++)
                    {
                        state_pub_->msg_.q[i] = q_.data(i);
                        state_pub_->msg_.qdot[i] = qdot_.data(i);
                        state_pub_->msg_.dq[i] = qd_.data(i);
                        state_pub_->msg_.dqdot[i] = qd_dot_.data(i);
                        state_pub_->msg_.effort_command[i] = act_torque(i);
                        state_pub_->msg_.Kp[i] = Kp_.data(i);
                        state_pub_->msg_.Kd[i] = Kd_.data(i);
                    }
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
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %f\n", t);
                printf("\n");

                printf("*** States in Joint Space (unit: deg) ***\n");

                for(int i=0; i < n_joints_; i++)
                {
                    printf("[%s]:  \t", joint_names_[i].c_str());
                    printf("Kp:%0.3lf, Kd:%0.3lf,\t", aKp_.data(i), aKd_.data(i));
                    printf("q: %0.3lf,\t", q_.data(i) * R2D);
                    printf("dq: %0.3lf,\t", qd_.data(i) * R2D);
                    printf("qdot: %0.3lf,\t", qdot_.data(i) * R2D);
                    printf("dqdot: %0.3lf,\t", qd_dot_.data(i) * R2D);
                    printf("tau: %0.3f\n", des_torque(i));
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
                    printf("no.%d, AngleAxis x: %0.2lf, y: %0.2lf, z: %0.2lf, Angle: %0.3lf\n\n",
                           j, ForwardAxis[j](0), ForwardAxis[j](1), ForwardAxis[j](2), ForwardAngle[j]);
                }
                printf("\n*********************************************************\n");
                count = 0;
                /*
                M_mat_collect.resize(16,16);
                M_mat_collect.setZero();
                M_mat_collect.block(0,0,2,2) = M_kdl_.data.block(0,0,2,2) + M1_kdl_.data.block(0,0,2,2);
                M_mat_collect.block(0,2,2,7) = M_kdl_.data.block(0,2,2,7);
                M_mat_collect.block(0,9,2,7) = M1_kdl_.data.block(0,2,2,7);
                M_mat_collect.block(2,0,7,2) = M_kdl_.data.block(2,0,7,2);
                M_mat_collect.block(9,0,7,2) = M1_kdl_.data.block(2,0,7,2);
                M_mat_collect.block(2,2,7,7) = M_kdl_.data.block(2,2,7,2);
                M_mat_collect.block(9,9,7,7) = M1_kdl_.data.block(2,2,7,2);
                std::cout << "M_kdl" << std::endl;
                std::cout << M_mat_collect << std::endl;
                std::cout << "M_PoE" << std::endl;
                std::cout << M << std::endl;
                */
                /*
                C_mat_collect.resize(16,16);
                C_mat_collect.setZero();
                C_mat_collect.block(0,0,2,2) = C_kdl_.data.block(0,0,2,2) + C1_kdl_.data.block(0,0,2,2);
                C_mat_collect.block(0,2,2,7) = C_kdl_.data.block(0,2,2,7);
                C_mat_collect.block(0,9,2,7) = C1_kdl_.data.block(0,2,2,7);
                C_mat_collect.block(2,0,7,2) = C_kdl_.data.block(2,0,7,2);
                C_mat_collect.block(9,0,7,2) = C1_kdl_.data.block(2,0,7,2);
                C_mat_collect.block(2,2,7,7) = C_kdl_.data.block(2,2,7,2);
                C_mat_collect.block(9,9,7,7) = C1_kdl_.data.block(2,2,7,2);
                std::cout << "C_kdl" << std::endl;
                std::cout << C_mat_collect << std::endl;
                std::cout << "C_PoE" << std::endl;
                std::cout << C << std::endl;
                */

            }
            count++;
        }

    private:
        // others
        double t;
        double InitTime=0;

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
        Eigen::MatrixXd C_mat_collect;

        KDL::Jacobian J1_kdl_, J2_kdl_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_, fk_pos_solver1_;     // Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_, jnt_to_jac_solver1_;    // Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_, id_solver1_;                          // Solver To compute the inverse dynamics

        MatrixXd M;
        VectorXd G;
        MatrixXd C;

        Vector3d ForwardPos[2];
        Vector3d ForwardOri[2];
        int NumChain=2;
        Vector3d ForwardAxis[2];
        double ForwardAngle[2];

        // Joint Space State
        KDL::JntArray qd_;
        KDL::JntArray qd_dot_;
        KDL::JntArray qd_ddot_;
        KDL::JntArray q_, q1_, q2_;
        KDL::JntArray qdot_, q1dot_, q2dot_;

        VectorXd dq;
        VectorXd des_torque;
        VectorXd act_torque;

        // Task Space State
        KDL::Frame x_[2];

        // gains
        KDL::JntArray Kp_, Ki_, Kd_, K_inf_;
        KDL::JntArray aKp_, aKi_, aKd_, aK_inf_;

        // publisher
        realtime_tools::RealtimeBuffer<std::vector<double>> pub_buffer_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<dualarm_controller::JointCurrentState>> state_pub_;

        // subscriber
        ros::Subscriber sub_x_cmd_;

        std::shared_ptr<SerialManipulator> cManipulator;
        std::unique_ptr<HYUControl::Controller> Control;
    };
}

PLUGINLIB_EXPORT_CLASS(dualarm_controller::JointSpace_Control,controller_interface::ControllerBase)