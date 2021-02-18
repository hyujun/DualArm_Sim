/*
 * KistHand.h
 *
 *  Created on: 2019. 12. 5.
 *      Author: Administrator
 */

#ifndef KISTHAND_H_
#define KISTHAND_H_

#include <Eigen/Dense>
using namespace Eigen;

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include "../KDL/PropertyDefinition.h"
#include "Trajectory.h"

#define R2D 180.0/M_PI
#define D2R M_PI/180.0

namespace HYUControl {

class KistHand {
public:
	KistHand();
	virtual ~KistHand();

	struct digit{
		float p[3];
		float q[4];
	};

	void HandEnctoRad( int *_enc, double *_rad );
	void HandVelocityConvert( int *_enc_s, double *_rad_s);
	void HandControl(int _Motion, double *_inc_enc, int32_t *_inc_command_vel, double _Time );


	void ForKinematics_Thumb(digit *pThumb);
	void ForKinematics_Fingers(digit *pFinger, int Pos );
	void InvKinematics_Fingers(digit *pFinger, int Pos );
	void InvKinematics_Thumb(digit *pThumb );

private:
	const float Lp = 20.21;
	const float Lb = 22.5;
	int L[4] = {17, 60, 36, 32};

	float Lt[5] = {84.05, 19.48, 47.0, 48.0, 35.0 };
	float qt = 45.0*D2R;

	float d[3] = {0, 0, 0};
	const float k= 0.707609;
	const float fth = M_PI/2.0;

	int HandMotion=0;
	int HandMotion_p=0;

	double ThumbFinger[4];
	double IndexFinger[3];
	//double MiddleFinger;

	int NewTarget = 0;
	int TotalDoF;
	double TrajectoryTime=0;



	Trajectory HandJointPoly5th;
	VectorXd targetpos;
	VectorXd dq;
	VectorXd q;
	VectorXd dqdot;
	VectorXd qdot;
	VectorXd dqddot;

	VectorXd K;
};

} /* namespace hyuCtrl */

#endif /* KISTHAND_H_ */
