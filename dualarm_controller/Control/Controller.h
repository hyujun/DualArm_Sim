#ifndef CONTROLLER_H_
#define CONTROLLER_H_

/**
 * @file Controller.h
 * @date 2019-05-15
 * @author Junho Park
 */

#define _USE_MATH_DEFINED
#include <cmath>
#include <Eigen/Dense>
#include "../KDL/PropertyDefinition.h"
#include "../KDL/SerialManipulator.h"
#include "../KDL/LieOperator.h"

#define KpBase 		20    	/**<Inital value of Kp*/
#define KdBase 		1		/**<Inital value of Kd*/
#define KiBase 		100		/**<Inital value of Ki*/
#define HinfBase 	5		/**<Inital value of H-infinity Gain*/

/**
 * @brief control and trajecotry namespace for manipulator
 * @version 2.0.0
 */
namespace HYUControl {

/**
 * @brief control algorithm for manipulator
 * @version 1.0.0
 */
class Controller : public HYUMotionBase::LieOperator {
public:
	/**
	 * @brief Controller constructor, JointNum is 6
	 */
	Controller();
	/**
	 * @brief Controller constructor
	 * @param[in] JointNum number of joint
	 */
	Controller(std::shared_ptr<SerialManipulator> Manipulator);
	virtual ~Controller();

	/**
	 * @brief make error member to be zero(e, e_dot, e_integral)
	 */
	void ClearError(void);

	/**
	 * @brief set the PID gains
	 * @param[in] Kp_ proportional gain
	 * @param[in] Kd_ derivative gain
	 * @param[in] Ki_ integral gain
	 * @param[in] JointNum number of joint
	 */
	void SetPIDGain(double &_Kp, double &_Kd, double &_Ki, int &_JointNum);
    void SetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki, VectorXd &_Kinf);
	void GetPIDGain(double *_Kp, double *_Kd, double *_Ki, int &_JointNum);
	void GetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki);
	void SetCLIKGain( const double &_Kp_Translation, const double &_Kp_Rotation );
	void SetTaskspaceGain( const VectorXd &_KpTask, const VectorXd &_KdTask);
	void GetTaskspaceGain( const VectorXd &_KpTask, const VectorXd &_KdTask);
	void SetImpedanceGain( const VectorXd &_Kp_Imp, const VectorXd &_Kd_Imp, const VectorXd &_Kp_Imp_Null, const VectorXd &_Kd_Imp_Null, const VectorXd &_des_m );
    void FrictionCompensator2( const VectorXd &_dqdot);
    void GetControllerStates(VectorXd &_dq, VectorXd &_dqdot, VectorXd &_ErrTask);
	/**
	 * @brief simple pd controller
	 * @param[in] q current joint position
	 * @param[in] q_dot current joint velocity
	 * @param[in] dq desired joint position
	 * @param[in] dq_dot desired joint velocity
	 * @param[in] toq joint input torque as control input
	 */
	void PDController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq);
	void PDGravController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq );
    void InvDynController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, const VectorXd &_dqddot, VectorXd &_Toq, const double &_dt );
    void InvDynController2( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, const VectorXd &_dqddot, VectorXd &_Toq, VectorXd &_frictionToq, const double &_dt );

    void TaskInvDynController( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_dxddot, const VectorXd &_q, const VectorXd &_qdot, VectorXd &_Toq, const double &_dt, const int mode);

	void TaskError( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot );
    void TaskError2( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot ,Quaterniond _q_R,Quaterniond _q_L);
	void TaskRelativeError( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot );

	void CLIKTaskController( const VectorXd &_q, const VectorXd &_qdot, Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_sensor, VectorXd &_Toq, const double &_dt, const int mode );

    void InertiaShaping( const VectorXd &_Mass, MatrixXd &_M_Shaped_inv );
	void TaskImpedanceController( const VectorXd &_q, const VectorXd &_qdot, Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_dxddot, const VectorXd &_sensor, VectorXd &_Toq, const int mode );
    void TaskImpedanceController2(const VectorXd &_q, const VectorXd &_qdot, Cartesiand *_dx,
                                               const VectorXd &_dxdot, const VectorXd &_dxddot, const VectorXd &_sensor,
                                               VectorXd &_Toq, Quaterniond &_q_R,Quaterniond &_q_L,const int mode);
	void FrictionIdentification( const VectorXd &_q, const VectorXd &_qdot, VectorXd &_dq, VectorXd &_dqdot, VectorXd &_dqddot, VectorXd &_Toq, const double &gt );
	void FrictionCompensator( const VectorXd &_qdot, const VectorXd &_dqdot );
	/**
	 * @brief joint input torque saturator
	 * @param[in] p_toq joint input torque as control input
	 * @param[in] maxtoq maximum torque motor can handle
	 * @param[in] p_dir direction of motor
	 */
	void OutputSaturation(double *pInput , double &_MaxInput);

private:
	Eigen::VectorXd Kp, KpTask;
	Eigen::VectorXd Kd, KdTask;
	Eigen::VectorXd Ki, KiTask;
	Eigen::VectorXd K_Hinf, K_HinfTask;

	Eigen::VectorXd KpImp, KdImp, KpImpNull, KdImpNull;
	Eigen::VectorXd mass_shaped;

	Eigen::VectorXd dq, dqdot, dqddot;
	Eigen::VectorXd dq_old;
	Eigen::VectorXd dqN, dqdotN;
	Eigen::VectorXd FrictionTorque;

	Eigen::VectorXd e, e_dev, e_int, e_int_sat;
	Eigen::VectorXd Vector_temp;
	Eigen::MatrixXd Matrix_temp;

	double alpha;

	Eigen::VectorXd eTask, edotTask;
	Eigen::MatrixXd edotTmp;

	Eigen::VectorXd GainWeightFactor;

	Eigen::MatrixXd ScaledTransJacobian;
	Eigen::MatrixXd pInvJacobian;
	Eigen::MatrixXd BlockpInvJacobian;
	Eigen::MatrixXd WdampedpInvJacobian;
	Eigen::MatrixXd DampedpInvJacobian;
	Eigen::MatrixXd AnalyticJacobian;
	Eigen::MatrixXd AnalyticJacobianDot;
	Eigen::MatrixXd RelativeJacobian;
	Eigen::MatrixXd RelativeJacobianDot;
    VectorXd q0dot;

    MatrixXd RelJacobian;
    MatrixXd dpInvRelJacobian;
    MatrixXd AJacwithRel;

	Eigen::MatrixXd M, Mx;
	Eigen::VectorXd G;

	int m_Jnum;
	double InitTime=0.0;

    std::shared_ptr<SerialManipulator> pManipulator;
};

} /* namespace HYUCtrl */

#endif /* CONTROLLER_H_ */
