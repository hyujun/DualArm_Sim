/*
 * Motion.cpp
 *
 *  Created on: 2019. 8. 1.
 *      Author: Administrator
 */

#include "Motion.h"

namespace HYUControl {

Motion::Motion() {

	this->pManipulator = NULL;
	TotalDoF=0;
	TotalChain=0;
	MotionProcess=0;
}

Motion::Motion(std::shared_ptr<SerialManipulator> Manipulator)
{
	this->pManipulator = Manipulator;

	TotalDoF = pManipulator->GetTotalDoF();
	TotalChain = pManipulator->GetTotalChain();

	MotionProcess=0;

	TargetPos.resize(TotalDoF);
	TargetPosTask.resize(7, TotalChain);
}

Motion::~Motion() {

}

uint16_t Motion::JointMotion(VectorXd &dq, VectorXd &dqdot, VectorXd &dqddot, VectorXd &_Target, const VectorXd &q, const VectorXd &qdot, double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType)
{
	this->MotionCommand = _MotionType;

	dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);

	if(_Time >= 0.1 && _StatusWord == SYSTEM_BEGIN)
	{
		//MotionCommand = MOVE_JOB;
		//MotionCommand = MOVE_CLIK_JOINT;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_ZERO && MotionProcess == MOVE_ZERO && _StatusWord == TARGET_ACHIEVED )
	{
		//MotionCommand = MOVE_CUSTOMIZE;
		//MotionCommand = MOVE_JOB;
		//MotionCommand = MOVE_FRICTION;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_JOB && MotionProcess == MOVE_JOB && _StatusWord == TARGET_ACHIEVED )
	{
		//MotionCommand = MOVE_CUSTOMIZE;
		//MotionCommand = MOVE_ZERO;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_JOINT_CYCLIC && MotionProcess == MOVE_JOINT_CYCLIC && (_Time >= MotionInitTime+20.0))
	{
		//MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if(JointPoly5th.isReady() == 0 && NewTarget==1)
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_ZERO;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(6) = -60.0*DEGtoRAD;
            TargetPos(13) = 60.0*DEGtoRAD;
			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_JOB ) //job posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_JOB;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(1) = -10.0*DEGtoRAD;

			TargetPos(3) = -20.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -0.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = 0.0*DEGtoRAD;
            TargetPos(5) = -TargetPos(5);
			TargetPos(6) = -70.00*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = 0.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE ) //pick the bottle
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE;
		}
		else
		{
			TargetPos.setZero();
            TargetPos = _Target;
			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE1) //hand
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE1;
		}
		else
		{
            init_time = _Time;

            TargetPos.setZero();
            TargetPos(0) = 0.57*DEGtoRAD;
            TargetPos(1) = -43.71*DEGtoRAD;

            TargetPos(2) = -41.26*DEGtoRAD;
            TargetPos(3) = -49.28*DEGtoRAD;
            TargetPos(4) = -25.86*DEGtoRAD;
            TargetPos(5) = -24.72*DEGtoRAD;
            TargetPos(6) = -83.19*DEGtoRAD;
            TargetPos(7) = -0.48*DEGtoRAD;
            TargetPos(8) = 0.56*DEGtoRAD;

            TargetPos(9) = 26.40*DEGtoRAD;
            TargetPos(10) = 43.45*DEGtoRAD;
            TargetPos(11) = 18.8*DEGtoRAD;
            TargetPos(12) = 28.26*DEGtoRAD;
            TargetPos(13) = 75.61*DEGtoRAD;
            TargetPos(14) = -3.78*DEGtoRAD;
            TargetPos(15) = -1.79*DEGtoRAD;

            _Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE2) //pick the bottle and stand-up
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE2;
		}
		else
		{
            init_time = _Time;

            TargetPos.setZero();
            TargetPos(0) = 0*DEGtoRAD;
            TargetPos(1) = 0*DEGtoRAD;

            TargetPos(2) = -5.0*DEGtoRAD;
            TargetPos(2+7) = -TargetPos(2);
            TargetPos(3) = -17.0*DEGtoRAD;
            TargetPos(3+7) = -TargetPos(3);
            TargetPos(4) = -3.33*DEGtoRAD;
            TargetPos(4+7) = -TargetPos(4);
            TargetPos(5) = 22.25*DEGtoRAD;
            TargetPos(5+7) = -TargetPos(5);
            TargetPos(6) = -83.27*DEGtoRAD;
            TargetPos(6+7) = 80.73*DEGtoRAD;
            TargetPos(7) = -10.0*DEGtoRAD;
            TargetPos(7+7) = -TargetPos(7);
            TargetPos(8) = 27.25*DEGtoRAD;
            TargetPos(8+7) = -TargetPos(8);

            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE3) //right put
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE3;
		}
		else
		{
			TargetPos.setZero();
            TargetPos(0) = 0*DEGtoRAD;
            TargetPos(1) = 0*DEGtoRAD;

            TargetPos(2) = -10*DEGtoRAD;
            TargetPos(3) = -80.0*DEGtoRAD;
            TargetPos(4) = -10.0*DEGtoRAD;
            TargetPos(5) = 80.50*DEGtoRAD;
            TargetPos(6) = -76.27*DEGtoRAD;
            TargetPos(7) = 23.20*DEGtoRAD;
            TargetPos(8) = 37.25*DEGtoRAD;

            TargetPos(2+7) = 20*DEGtoRAD;
            TargetPos(3+7) = 17.0*DEGtoRAD;
            TargetPos(4+7) = -0.33*DEGtoRAD;
            TargetPos(5+7) = -14.25*DEGtoRAD;
            TargetPos(6+7) = 88.27*DEGtoRAD;
            TargetPos(7+7) = 10.0*DEGtoRAD;
            TargetPos(8+7) = -27.25*DEGtoRAD;
			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_JOINT_CYCLIC )
	{
		if( MotionCommand != MotionCommand_p )
		{
			MotionInitTime = _Time;
		}
		else
		{
			_T = 18.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70;

			dq(0) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(0) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(0) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			_T = 10.0;
			_omega = 2.0*M_PI/_T;
			_amp = 20;

			dq(1) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.8481) - 15*M_PI/180.0;
			dqdot(1) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.8481);
			dqddot(1) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.8481);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(2) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.6435) - 30*M_PI/180.0;
			dqdot(2) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.6435);
			dqddot(2) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.6435);

			dq(2+6) = -dq(2);
			dqdot(2+6) = -dqdot(2);
			dqddot(2+6) = -dqddot(2);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(3) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.4115) - 20*M_PI/180.0;
			dqdot(3) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.4115);
			dqddot(3) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.4115);

			dq(3+6) = -dq(3);
			dqdot(3+6) = -dqdot(3);
			dqddot(3+6) = -dqddot(3);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 45;

			dq(4) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.729) - 30*M_PI/180.0;
			dqdot(4) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.729);
			dqddot(4) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.729);

			dq(4+6) = -dq(4);
			dqdot(4+6) = -dqdot(4);
			dqddot(4+6) = -dqddot(4);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70.0;

			dq(5) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(5) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(5) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(5+6) = -dq(5);
			dqdot(5+6) = -dqdot(5);
			dqddot(5+6) = -dqddot(5);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 40.0;

			dq(6) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(6) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(6) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(6+6) = -dq(6);
			dqdot(6+6) = -dqdot(6);
			dqddot(6+6) = -dqddot(6);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 40.0;

			dq(7) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(7) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(7) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(7+6) = dq(7);
			dqdot(7+6) = dqdot(7);
			dqddot(7+6) = dqddot(7);

			MotionProcess = MOVE_JOINT_CYCLIC;
		}
	}

	MotionCommand_p = MotionCommand;
	return MotionProcess;
}

uint16_t Motion::TaskMotion( VectorXd &_dx, VectorXd &_dxdot, VectorXd &_dxddot, VectorXd _Target, const VectorXd &x, const VectorXd &qdot, double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType )
{
	MotionCommandTask = _MotionType;
    int target_size = 12;

	pManipulator->pKin->GetAnalyticJacobian(AJacobian);
	xdot.setZero(target_size);
	xdot.noalias() += AJacobian*qdot;

    auto A = 0.12;
    auto b1 = 0.64;
    auto b2 = -0.33;
    auto b3 = 0.45;
    auto f = 0.2;

    auto l_p1 = 0.58;
    auto l_p2 = 0.33;
    auto l_p3 = 0.42;

    if( MotionCommandTask == MOVE_TASK_CUSTOM )
    {
        if( _StatusWord != TARGET_ACHIEVED )
        {
            if( TaskPoly5th.isReady()==0 && NewTarget==1 )
            {
                TaskPoly5th.SetPoly5th(_Time, x, xdot, TargetPosTask, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx, _dxdot, _dxddot);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx, _dxdot, _dxddot);
            }

            MotionProcess = MOVE_TASK_CUSTOM;
        }
        else
        {
            TargetPosTask = _Target;
            TrajectoryTime=5.0;
            NewTarget=1;
            _StatusWord = TARGET_MOVING;
        }
    }
	else if( MotionCommandTask == MOVE_TASK_CUSTOM1 )
	{
        if( MotionCommand != MotionCommand_p )
        {
            MotionInitTime = _Time;
        }
        else
        {
            _dx(0) = 0;
            _dx(1) = 0;
            _dx(2) = 0;
            _dx(3) = A * sin(f * M_PI * (_Time - MotionInitTime)) + b1;
            _dx(4) = b2;
            _dx(5) = b3;

            _dx(6) = 0;
            _dx(7) = 0;
            _dx(8) = 0;
            _dx(9) = -A * sin(f * M_PI * (_Time - MotionInitTime)) + l_p1;
            _dx(10) = l_p2;
            _dx(11) = l_p3;

            _dxdot.setZero(12);
            _dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = -(f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));

            MotionProcess = MOVE_TASK_CUSTOM1;
		}
	}
    else if( MotionCommandTask == MOVE_TASK_CUSTOM2 )
    {
        if( MotionCommand != MotionCommand_p )
        {
            MotionInitTime = _Time;
        }
        else
        {
            _dx(0) = 0;
            _dx(1) = 0;
            _dx(2) = 0;
            _dx(3) = b1;
            _dx(4) = A * sin(f * M_PI * (_Time - MotionInitTime)) + b2;
            _dx(5) = b3;

            _dx(6) = 0;
            _dx(7) = 0;
            _dx(8) = 0;
            _dx(9) = l_p1;
            _dx(10) = -A * sin(f * M_PI * (_Time - MotionInitTime)) + l_p2;
            _dx(11) = l_p3;

            _dxdot.setZero(12);
            _dxdot(4) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));

            MotionProcess = MOVE_TASK_CUSTOM2;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM3 )
    {
        if( MotionCommand != MotionCommand_p )
        {
            MotionInitTime = _Time;
        }
        else
        {
            A = 0.07;
            _dx(0) = 0;
            _dx(1) = 0;
            _dx(2) = 0;
            _dx(3) = A * sin(f * M_PI * (_Time - MotionInitTime)) + b1 + 0.09;
            _dx(4) = b2 + 0.11;
            _dx(5) = b3;

            _dx(6) = 0;
            _dx(7) = 0;
            _dx(8) = 0;
            _dx(9) = -A * cos(2*f * M_PI * (_Time - MotionInitTime));
            _dx(10) = -A * sin(2*f * M_PI * (_Time - MotionInitTime))+0.4;
            _dx(11) = 0;

            _dxdot.setZero(12);
            _dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = (2*f * M_PI) * A * sin(2*f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(2*f * M_PI) * A * cos(2*f * M_PI * (_Time - MotionInitTime));

            MotionProcess = MOVE_TASK_CUSTOM3;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM4 )
    {
        if( MotionCommand != MotionCommand_p )
        {
            MotionInitTime = _Time;
        }
        else
        {
            _dx(0) = 0.0*DEGtoRAD;
            _dx(1) = 0.0*DEGtoRAD;
            _dx(2) = 0.0*DEGtoRAD;
            _dx(3) = b1;
            _dx(4) = b2;
            _dx(5) = b3;

            _dx(6) = 0.0*DEGtoRAD;
            _dx(7) = 0.0*DEGtoRAD;
            _dx(8) = 0.0*DEGtoRAD;
            _dx(9) = l_p1;
            _dx(10) = l_p2;
            _dx(11) = l_p3;

            _dxdot.setZero(12);
            MotionProcess = MOVE_TASK_CUSTOM4;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM5 )
    {
        if( MotionCommand != MotionCommand_p )
        {
            MotionInitTime = _Time;
        }
        else
        {
            _dx = _Target;
            _dxdot.setZero(12);
            MotionProcess = MOVE_TASK_CUSTOM5;
        }
    }

	MotionCommandTask_p = MotionCommandTask;
	return MotionProcess;
}

} /* namespace hyuCtrl */
