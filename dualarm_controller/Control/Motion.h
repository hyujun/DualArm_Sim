/*
 * Motion.h
 *
 *  Created on: 2019. 8. 1.
 *      Author: Administrator
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "Trajectory.h"
#include "../KDL/SerialManipulator.h"
#include "../KDL/LieOperator.h"

namespace HYUControl {

class Motion {
public:
	Motion();
	explicit Motion(SerialManipulator *_pManipulator);
	virtual ~Motion();

	uint16_t JointMotion(double *_dq, double *_dqdot, double *_dqddot, VectorXd &_Target, double *_q, double *_qdot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType);
	uint16_t TaskMotion( VectorXd *_dx, VectorXd *_dxdot, VectorXd *_dxddot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType );

private:

	Eigen::VectorXd e, edot;
	Eigen::VectorXd q, qdot;
	Eigen::VectorXd dq, dqdot, dqddot;
	Eigen::VectorXd TargetPos;

	Eigen::MatrixXd omega;

	Eigen::MatrixXd x;
	Eigen::MatrixXd xdot;
	Eigen::MatrixXd TargetPosTask;

	SerialManipulator *pManipulator;

	Trajectory JointPoly5th;
	Trajectory TaskPoly5th;

	int MotionProcess;
	int NewTarget=0;

	uint16_t MotionCommand=0;
	uint16_t MotionCommand_p=0;
	uint16_t MotionCommandTask=0;
	uint16_t MotionCommandTask_p=0;

	double MotionInitTime=0;
	double TrajectoryTime=0;

	double init_time=0;

	int TotalDoF;
	int TotalChain;

	double _T=0;
	double _omega=0;
	double _amp=0;

	int MotionCounter=0;

};

} /* namespace hyuCtrl */

#endif /* MOTION_H_ */
