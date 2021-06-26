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
    MotionCommand_p = 0;

	TargetPos.setZero(TotalDoF);
    TargetPos_p.setZero(TotalDoF);

	TargetPosTask.setZero(12);
    TargetPosTask_p.setZero(12);

	TargetPos_Linear.setZero(6);

}

Motion::~Motion() {

}

uint16_t Motion::JointMotion(VectorXd &dq, VectorXd &dqdot, VectorXd &dqddot,
                             VectorXd &_Target, const VectorXd &q, const VectorXd &qdot,
                             double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType)
{
	this->MotionCommand = _MotionType;

	dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( _StatusWord != MotionCommand )
		{
			if( NewTarget==1 )
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
			TargetPos.setZero(16);
			TargetPos(6) = -60.0*DEGtoRAD;
            TargetPos(13) = 60.0*DEGtoRAD;
			TrajectoryTime=5.0;
			NewTarget=1;
            _Target = TargetPos;
			_StatusWord = 0;
		}
	}
	else if( MotionCommand == MOVE_JOB ) //job posture
	{
        if( _StatusWord != MotionCommand )
		{
			if( NewTarget==1 )
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
			TargetPos.setZero(16);
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
			_StatusWord = 0;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE )
	{
        if( _StatusWord != MotionCommand )
		{
			if( NewTarget == 1 )
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
            TargetPos_p = TargetPos;
			TrajectoryTime=5.0;
			NewTarget=1;
			_StatusWord = 0;
		}
	}
	else if( MotionCommand == MOVE_JOINT_CYCLIC )
	{
        if( _StatusWord == MotionCommand )
		{
			MotionInitTime = _Time;
			_StatusWord = 0;
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

uint16_t Motion::TaskMotion( VectorXd &_dx, VectorXd &_dxdot, VectorXd &_dxddot,
                             VectorXd _Target, const VectorXd &x, const VectorXd &qdot,
                             double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType )
{
	MotionCommandTask = _MotionType;

	pManipulator->pKin->GetAnalyticJacobian(AJacobian);
	xdot.setZero(12);
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
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);

                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx.setZero(12);
            _dxdot.setZero(12);
            _dxddot.setZero(12);

            _dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM;
        }
        else
        {
            TargetPosTask = _Target;
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
	else if( MotionCommandTask == MOVE_TASK_CUSTOM1 )
	{
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = -A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(9);
            _dx(10) = start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = -(f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(3) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(9) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
            _StatusWord = 0;
		}
	}
    else if( MotionCommandTask == MOVE_TASK_CUSTOM2 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = start_pos(9);
            _dx(10) = -A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(4) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(4) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(10) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM3 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            auto ref_A = 0.1;
            auto rel_A = 0.05;
            f = 1.0/10.0;
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = -ref_A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = -rel_A * cos(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(9);
            _dx(10) = -rel_A * sin(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(3) = -(f * M_PI) * ref_A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(3) = (f * M_PI) * (f * M_PI) * ref_A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(9) = (2*f * M_PI) * (2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));
            _dxddot(10) = (2*f * M_PI) * (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));

        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM4 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            auto ref_A = 0.40;
            auto rel_A = 0.05;
            f = 1.0/35.0;
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = -ref_A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(4);;
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = -rel_A * cos(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(9);
            _dx(10) = -rel_A * sin(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(4) = -(f * M_PI) * ref_A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(4) = (f * M_PI) * (f * M_PI) * ref_A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(9) = (2*f * M_PI) * (2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));
            _dxddot(10) = (2*f * M_PI) * (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));

        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM5 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            auto ref_A = 0.45;
            auto rel_A = 0.1;
            f = 1.0/30.0;
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = start_pos(4);;
            _dx(5) = -ref_A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = -rel_A * cos(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(9);
            _dx(10) = -rel_A * sin(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(5) = -(f * M_PI) * ref_A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(5) = (f * M_PI) * (f * M_PI) * ref_A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(9) = (2*f * M_PI) * (2*f * M_PI) * rel_A * cos(2*f * M_PI * (_Time - MotionInitTime));
            _dxddot(10) = (2*f * M_PI) * (2*f * M_PI) * rel_A * sin(2*f * M_PI * (_Time - MotionInitTime));


        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM6 )
    {
        if( _StatusWord != MotionCommandTask )
        {
            _dx.setZero(12);
            _dxdot.setZero(12);
            _dxddot.setZero(12);

            if( NewTarget==1 )
            {
                int target_size = 6;
                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);

                Matrix3d SO3_a;
                Matrix3d SO3_d;
                pManipulator->pKin->RollPitchYawtoSO3(_dx(0), _dx(1), _dx(2), SO3_d);
                pManipulator->pKin->RollPitchYawtoSO3(x(0), x(1), x(2), SO3_a);
                slerp_ori.slerp_setup(SO3_d, SO3_a, _Time, TrajectoryTime);
                slerp_ori.slerp_profile(_dxddot.head(3), _dxdot.head(3), _dx.head(3), _Time);

                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                slerp_ori.slerp_profile(_dxddot.head(3), _dxdot.head(3), _dx.head(3), _Time);
            }

            //_dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM6;
        }
        else
        {
            TargetPosTask = _Target;
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }

	MotionCommandTask_p = MotionCommandTask;
	return MotionProcess;
}

} /* namespace hyuCtrl */
