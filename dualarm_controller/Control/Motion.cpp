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

uint16_t Motion::JointMotion(VectorXd &dq, VectorXd &dqdot, VectorXd &dqddot, VectorXd &_Target, const VectorXd &q, const VectorXd &qdot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType)
{
	this->MotionCommand = _MotionType;

	dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);

	if(_Time >= 1.0 && _StatusWord == SYSTEM_BEGIN)
	{
		MotionCommand = MOVE_JOB;
		//MotionCommand = MOVE_CLIK_JOINT;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_ZERO && MotionProcess == MOVE_ZERO && _StatusWord == TARGET_ACHIEVED )
	{
		MotionCommand = MOVE_CUSTOMIZE;
		//MotionCommand = MOVE_JOB;
		//MotionCommand = MOVE_FRICTION;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_JOB && MotionProcess == MOVE_JOB && _StatusWord == TARGET_ACHIEVED )
	{
		MotionCommand = MOVE_CUSTOMIZE;
		//MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_CUSTOMIZE && MotionProcess == MOVE_CUSTOMIZE && _StatusWord == TARGET_ACHIEVED )
	{
	    if(MotionCounter == 0)
        {
            MotionCommand = MOVE_CUSTOMIZE1;
        }
	    else if(MotionCounter == 2)
        {
            MotionCommand = MOVE_CUSTOMIZE9;
        }
	    else
        {
            MotionCommand = MOVE_JOB;
        }
        MotionCounter=0;
		_MotionType = MotionCommand;
	}
    else if( MotionCommand == MOVE_CUSTOMIZE1 && MotionProcess == MOVE_CUSTOMIZE1 && _StatusWord == TARGET_ACHIEVED ) //hand
    {
        if(_Time - init_time >= 7.0)
        {
            MotionCommand = MOVE_CUSTOMIZE1;
            //MotionCommand = MOVE_JOB;
            _MotionType = MotionCommand;
        }
        else
        {
            MotionCommand = MOVE_CUSTOMIZE2;
            //MotionCommand = MOVE_JOB;
            _MotionType = MotionCommand;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE2 && MotionProcess == MOVE_CUSTOMIZE2 && _StatusWord == TARGET_ACHIEVED )
    {
        if(MotionCounter == 0)
        {
            MotionCommand = MOVE_CUSTOMIZE3;
        }
        else if(MotionCounter == 2)
        {
            MotionCommand = MOVE_CUSTOMIZE;
        }
        else
        {
            MotionCommand = MOVE_CUSTOMIZE4;
        }
        //MotionCommand = MOVE_JOB;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE3 && MotionProcess == MOVE_CUSTOMIZE3 && _StatusWord == TARGET_ACHIEVED )
    {
        if(MotionCounter == 0)
        {
            MotionCommand = MOVE_CUSTOMIZE2;
            MotionCounter++;
        }
        else
        {
            MotionCommand = MOVE_CUSTOMIZE4;
        }
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE4 && MotionProcess == MOVE_CUSTOMIZE4 && _StatusWord == TARGET_ACHIEVED )
    {
        //MotionCommand = MOVE_CUSTOMIZE5;

        if(MotionCounter != 0)
        {
            MotionCommand = MOVE_CUSTOMIZE2;
            MotionCounter++;
        }
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE5 && MotionProcess == MOVE_CUSTOMIZE5 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE6;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE6 && MotionProcess == MOVE_CUSTOMIZE6 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE7;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE7 && MotionProcess == MOVE_CUSTOMIZE7 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE8;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE8 && MotionProcess == MOVE_CUSTOMIZE8 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE9;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE9 && MotionProcess == MOVE_CUSTOMIZE9 && _StatusWord == TARGET_ACHIEVED ) //hand
    {
        if(_Time - init_time >= 7.0)
        {
            MotionCommand = MOVE_CUSTOMIZE9;
            _MotionType = MotionCommand;
        }
        else
        {
            MotionCommand = MOVE_JOB;
            _MotionType = MotionCommand;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE10 && MotionProcess == MOVE_CUSTOMIZE10 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE11;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE11 && MotionProcess == MOVE_CUSTOMIZE11 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE12;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE12 && MotionProcess == MOVE_CUSTOMIZE12 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE13;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE13 && MotionProcess == MOVE_CUSTOMIZE13 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE14;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE14 && MotionProcess == MOVE_CUSTOMIZE14 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE15;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE15 && MotionProcess == MOVE_CUSTOMIZE15 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE16;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE16 && MotionProcess == MOVE_CUSTOMIZE16 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_CUSTOMIZE17;
        _MotionType = MotionCommand;
    }
    else if( MotionCommand == MOVE_CUSTOMIZE17 && MotionProcess == MOVE_CUSTOMIZE17 && _StatusWord == TARGET_ACHIEVED )
    {
        MotionCommand = MOVE_JOB;
        _MotionType = MotionCommand;
    }
	else if( MotionCommand == MOVE_JOINT_CYCLIC && MotionProcess == MOVE_JOINT_CYCLIC && (_Time >= MotionInitTime+20.0))
	{
		MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if(JointPoly5th.isReady() == 0 && NewTarget==1)
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_ZERO;
		}
		else
		{
			TargetPos.setZero();

			_Target = TargetPos;

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
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_JOB;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(1) = -0.0*DEGtoRAD;

			TargetPos(3) = -15.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -20.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = 0.0*DEGtoRAD;
            TargetPos(5) = -TargetPos(5);
			TargetPos(6) = -80.00*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = 20.0*DEGtoRAD;
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
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE;
		}
		else
		{
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
	else if( MotionCommand == MOVE_CUSTOMIZE1) //hand
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
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
    else if( MotionCommand == MOVE_CUSTOMIZE4) //left put
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE4;
        }
        else
        {
            init_time = _Time;

            TargetPos.setZero();
            TargetPos(0) = 0*DEGtoRAD;
            TargetPos(1) = 0*DEGtoRAD;

            TargetPos(2) = -15*DEGtoRAD;
            TargetPos(3) = -20.0*DEGtoRAD;
            TargetPos(4) = 0.33*DEGtoRAD;
            TargetPos(5) = 15.25*DEGtoRAD;
            TargetPos(6) = -80.27*DEGtoRAD;
            TargetPos(7) = -10.0*DEGtoRAD;
            TargetPos(8) = 27.25*DEGtoRAD;

            TargetPos(2+7) = 16.0*DEGtoRAD;
            TargetPos(3+7) = 65.0*DEGtoRAD;
            TargetPos(4+7) = 10.0*DEGtoRAD;
            TargetPos(5+7) = -85.31*DEGtoRAD;
            TargetPos(6+7) = 68.27*DEGtoRAD;
            TargetPos(7+7) = -20.69*DEGtoRAD;
            TargetPos(8+7) = -37.53*DEGtoRAD;

            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE5)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE5;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE6)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE6;
        }
        else
        {
            TargetPos.setZero();

            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE7)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE7;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE8)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE8;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE9)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE9;
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
    else if( MotionCommand == MOVE_CUSTOMIZE10)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE10;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE11)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE11;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE12) //gym2
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE12;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE13)
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE13;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE14) //gym2
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE14;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE15) //gym2
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE15;
        }
        else
        {
            TargetPos.setZero();


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE16) //gym2
    {
        if( MotionCommand == MotionCommand_p )
        {
            if( JointPoly5th.isReady() == 0 && NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE16;
        }
        else
        {
            TargetPos.setZero();

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

uint16_t Motion::TaskMotion( VectorXd *_dx, VectorXd *_dxdot, VectorXd *_dxddot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType )
{
	MotionCommandTask = _MotionType;

	//int TotalTask = 4*TotalChain;

	if( MotionCommandTask == MOVE_CLIK_JOINT )
	{
		if( MotionCommandTask == MotionCommandTask_p )
		{
			//if( TaskPoly5th.isReady()==0 && NewTarget==1 )
			//{
			//	TaskPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalTask);
			//	NewTarget=0;
			//}
			//else
			//{
			//	TaskPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
			//}

			for(int i=0; i<TotalChain; i++)
			{
				//_dx[i].resize(7);
				_dx[i].head(3) = TargetPosTask.block(0,i,3,1);
				_dx[i](3) = TargetPosTask(3,i);
				_dx[i].tail(3) = TargetPosTask.block(4,i,3,1);

				_dxdot[i].setZero();
				_dxddot[i].setZero();
			}

			MotionProcess = MOVE_CLIK_JOINT;
		}
		else
		{
			TargetPosTask.setZero();
			TargetPosTask.block(0,0,3,1) << 0, -1, 0;
			TargetPosTask.block(3,0,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,0,3,1) << 0.310, -0.310, 0.420;

			TargetPosTask.block(0,1,3,1) << 0, -1, 0;
			TargetPosTask.block(3,1,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,1,3,1) << 0.310, 0.310, 0.420;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}

	MotionCommandTask_p = MotionCommandTask;

	return MotionProcess;
}

} /* namespace hyuCtrl */
