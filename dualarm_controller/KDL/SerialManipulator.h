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
#include "motionlibrary.h"

#define RADtoDEG 180/M_PI		/**<constant of radian to degree*/
#define DEGtoRAD M_PI/180		/**<constant of degree to radian*/

#define sign(a) (((a)<0) ? -1 : ((a)>0))  /**<Definition of Signum function*/

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

    void StateMachine( const VectorXd _q, const VectorXd _qdot, VectorXd &_Target, unsigned char &_StateWord, unsigned char &_ControlWord );

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
