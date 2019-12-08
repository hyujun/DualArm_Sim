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
#include "PropertyDefinition.h"
#include "SerialManipulator.h"
#include "LieOperator.h"

#define KpBase 		100    	/**<Inital value of Kp*/
#define KdBase 		20		/**<Inital value of Kd*/
#define HinfBase 	5		/**<Inital value of H-infinity Gain*/

/**
 * @brief control and trajecotry namespace for manipulator
 * @version 1.0.0
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
	Controller(SerialManipulator *pManipulator);
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
	void SetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki);
	void GetPIDGain(double *_Kp, double *_Kd, double *_Ki, int &_JointNum);
	void GetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki);
	/**
	 * @brief simple pd controller
	 * @param[in] q current joint position
	 * @param[in] q_dot current joint velocity
	 * @param[in] dq desired joint position
	 * @param[in] dq_dot desired joint velocity
	 * @param[in] toq joint input torque as control input
	 */
	void PDController( double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_Toq, double &_dt );
	void PDGravController( double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_Toq, double &_dt );
	void InvDynController( VectorXd &_q, VectorXd &_qdot, VectorXd &_dq, VectorXd &_dqdot, VectorXd &_dqddot, double *p_Toq, double &_dt );

	void CLIKTaskController( double *_q, double *_qdot, double *_dq, double *_dqdot, const VectorXd *_dx, const VectorXd *_dxdot, const VectorXd &_dqdotNull, double *p_Toq, double &_dt );

	void FrictionIdentification( double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_dqddot, double *p_Toq, double &gt );
	void FrictionCompensator( VectorXd &_qdot, VectorXd &_dqdot );
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
	Eigen::VectorXd K_Hinf, KiTask;

	Eigen::VectorXd q, dq, qdot, dqdot, dqddot;
	Eigen::VectorXd FrictionTorque;

	Eigen::VectorXd e, e_dev, e_int, e_int_sat;

	SE3 dSE3, eSE3;
	Eigen::Vector3d  omega;
	double theta=0;

	Eigen::VectorXd eTask, edotTask;
	Eigen::MatrixXd edotTmp;
	Eigen::MatrixXd dexp;

	Eigen::VectorXd ToqOut;
	Eigen::VectorXd GainWeightFactor;

	Eigen::MatrixXd ScaledTransJacobian;
	Eigen::MatrixXd BodyJacobian;
	Eigen::MatrixXd AnalyticJacobian;

	Eigen::MatrixXd M, Mx;
	Eigen::VectorXd G, Gx;

	int m_Jnum;
	double m_KpBase, m_KdBase, m_HinfBase;
	double InitTime=0;

	SerialManipulator *pManipulator;
};

} /* namespace HYUCtrl */

#endif /* CONTROLLER_H_ */
