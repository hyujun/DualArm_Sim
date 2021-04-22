/**
 * @file SerialManipulator.h
 * @date 2019-09-17
 * @author Junho Park
 */

#ifndef SERIALMANIPULATOR_H_
#define SERIALMANIPULATOR_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
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

#define MOVE_CUSTOMIZE1 0x0008
#define MOVE_CUSTOMIZE2 0x0009
#define MOVE_CUSTOMIZE3 0x000a
#define MOVE_CUSTOMIZE4 0x000b
#define MOVE_CUSTOMIZE5 0x000c
#define MOVE_CUSTOMIZE6 0x000d
#define MOVE_CUSTOMIZE7 0x000e
#define MOVE_CUSTOMIZE8 0x0010
#define MOVE_CUSTOMIZE9 0x0011
#define MOVE_CUSTOMIZE10 0x0012
#define MOVE_CUSTOMIZE11 0x0013
#define MOVE_CUSTOMIZE12 0x0014
#define MOVE_CUSTOMIZE13 0x0015
#define MOVE_CUSTOMIZE14 0x0016
#define MOVE_CUSTOMIZE15 0x0017
#define MOVE_CUSTOMIZE16 0x0018
#define MOVE_CUSTOMIZE17 0x0019
#define MOVE_CUSTOMIZE18 0x001a
#define MOVE_CUSTOMIZE19 0x001b
#define MOVE_CUSTOMIZE20 0x001c


#define TARGET_MOVING 0x0010
#define TARGET_ACHIEVED 0x0050
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

    std::unique_ptr<HYUMotionDynamics::Liedynamics> pDyn;
    std::unique_ptr<HYUMotionKinematics::PoEKinematics> pKin;

    void StateMachine( const VectorXd _q, const VectorXd _qdot, VectorXd &_Target, uint16_t &_StateWord, uint16_t &_ControlWord );

    /**
     * @brief update robot kinematics & dynamics information
     */
	void UpdateManipulatorParam();

	/**
	 * @brief convert encoder inc to generalized coordinate joint position q(rad)
	 */
	void ENCtoRAD( int *_enc, VectorXd &_rad );

	/**
	 * @brief convert encoder generalized coordinate joint position q(rad) to encoder inc
	 */
	void RADtoENC( int *_enc, VectorXd &_rad );

	/**
	 * @brief convert encoder inc/second to generalized coordinate joint velocity q_dot(rad/second)
	 */
	void VelocityConvert( int32_t *_enc_sec, VectorXd &_rad_sec );

	void TorqueConvert( VectorXd &_Torque, short *_pOutput, short &_MaxOutput );

	int GetTotalDoF(void)
	{
		return mChainMat.cols();
	}

	int GetTotalChain(void)
	{
		return mChainMat.rows();
	}

private:
    Vector3d w[16]; 	/**< kinematic information container */
    Vector3d p[16]; 	/**< kinematic information container */
    Vector3d L[16]; 	/**< kinematic information container */
    Vector3d CoM[16];
    Matrix3d Iner[16];
    double mass[16];

    MatrixXi mChainMat;

    int mDoF_Total;
    int mChain_Total;

    uint16_t mState_now=0;
    uint16_t mState_pre=0;

    VectorXd q;
    VectorXd qdot;
};

#endif /* SERIALMANIPULATOR_H_ */
