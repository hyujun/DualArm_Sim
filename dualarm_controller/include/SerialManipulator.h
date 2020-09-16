/**
 * @file SerialManipulator.h
 * @date 2019-09-17
 * @author Junho Park
 */

#ifndef SERIALMANIPULATOR_H_
#define SERIALMANIPULATOR_H_

#define _USE_MATH_DEFINES
#include <math.h>

#include "LieDynamics.h"
#include "PoEKinematics.h"
#include "PropertyDefinition.h"

#define RADtoDEG 180/M_PI		/**<constant of radian to degree*/
#define DEGtoRAD M_PI/180		/**<constant of degree to radian*/

#define sign(a) (((a)<0) ? -1 : ((a)>0))  /**<Definition of Signum function*/

#define MOVE_ZERO 0x0001
#define MOVE_JOB 0x0002
#define MOVE_JOINT_CYCLIC 0x0003
#define MOVE_TASK_CYCLIC 0x0004
#define MOVE_CLIK_JOINT 0x0005
#define MOVE_CUSTOMIZE 0x0006
#define MOVE_FRICTION 0x0007

#define TARGET_MOVING 0x0010
#define TARGET_ZERO 0x0020
#define TARGET_JOB 0x0030
#define TARGET_CUSTOMIZE 0x0040

#define SYSTEM_BEGIN 0xffff
/**
 * @brief Seria lManipulator info update
 * @date 2017-10-23
 * @version 1.0.0
 */
class SerialManipulator
{
public:

	SerialManipulator();
	virtual ~SerialManipulator();

    HYUMotionDynamics::Liedynamics *pDyn;
    HYUMotionKinematics::PoEKinematics *pKin;
    HYUMotionKinematics::PoEKinematics *pCoMKin;

    void StateMachine( double *_q, double *_qdot, VectorXd &_Target, uint16_t &_StateWord, uint16_t &_ControlWord );

    /**
     * @brief update robot kinematics & dynamics information
     */
	void UpdateManipulatorParam(void);

	/**
	 * @brief convert encoder inc to generalized coordinate joint position q(rad)
	 */
	void ENCtoRAD(int *_enc, double *_rad );

	/**
	 * @brief convert encoder generalized coordinate joint position q(rad) to encoder inc
	 */
	void RADtoENC(int *_enc, double *_rad);

	/**
	 * @brief convert encoder inc/second to generalized coordinate joint velocity q_dot(rad/second)
	 */
	void VelocityConvert( int32_t *_enc_sec, double *_rad_sec );

	void TorqueConvert( double *_pTorque, short *_pOutput, short &_MaxOutput );

	int GetTotalDoF(void)
	{
		return mChainMat.cols();
	}

	int GetTotalChain(void)
	{
		return mChainMat.rows();
	}

private:
    Vector3d w[15]; 	/**< kinematic information container */
    Vector3d p[15]; 	/**< kinematic information container */
    Vector3d L[15]; 	/**< kinematic information container */
    Vector3d CoM[15];

    Matrix3d Iner[15];
    double mass[15];

    MatrixXi mChainMat;

    int mDoF_Total;
    int mChain_Total;

    uint16_t mState_now=0;
    uint16_t mState_pre=0;

    VectorXd q, qdot;
};

#endif /* SERIALMANIPULATOR_H_ */
