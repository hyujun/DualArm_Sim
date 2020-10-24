#include "SerialManipulator.h"

SerialManipulator::SerialManipulator()
{
	mChainMat.resize(2,16);
	mChainMat << 1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,
				1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1;

	this->mDoF_Total = mChainMat.cols();
	this->mChain_Total = mChainMat.rows();


	pKin = new HYUMotionKinematics::PoEKinematics(mChainMat);
	pCoMKin = new HYUMotionKinematics::PoEKinematics(mChainMat);
	pDyn = new HYUMotionDynamics::Liedynamics(mChainMat, *pKin, *pCoMKin);

}

SerialManipulator::~SerialManipulator()
{
	delete pDyn;
	delete pCoMKin;
	delete pKin;
}

void SerialManipulator::StateMachine( double *_q, double *_qdot, VectorXd &_Target, uint16_t &_StateWord, uint16_t &_ControlWord )
{
	q = Map<VectorXd>(_q, mDoF_Total);
	qdot = Map<VectorXd>(_qdot, mDoF_Total);

	mState_now = _StateWord;

	if( _ControlWord == SYSTEM_BEGIN )
	{
		mState_now = SYSTEM_BEGIN;
	}
	else if( _ControlWord == MOVE_ZERO )
	{
		q -= _Target;

		if((q.cwiseAbs().maxCoeff() <= 1.2*DEGtoRAD) && (qdot.cwiseAbs().maxCoeff() <= 0.5*DEGtoRAD))
			mState_now = TARGET_ZERO;
		else
			mState_now = TARGET_MOVING;
	}
	else if( _ControlWord == MOVE_JOB )
	{
		q -= _Target;

		if((q.cwiseAbs().maxCoeff() <= 1.2*DEGtoRAD) && (qdot.cwiseAbs().maxCoeff() <= 0.5*DEGtoRAD))
			mState_now = TARGET_JOB;
		else
			mState_now = TARGET_MOVING;
	}
	else if( _ControlWord == MOVE_CUSTOMIZE )
	{
		q -= _Target;
		if((q.cwiseAbs().maxCoeff() <= 1.2*DEGtoRAD) && (qdot.cwiseAbs().maxCoeff() <= 0.5*DEGtoRAD))
			mState_now = TARGET_CUSTOMIZE;
		else
			mState_now = TARGET_MOVING;

	}
	else
	{
		mState_now = TARGET_MOVING;
	}

	mState_pre = mState_now;
	_StateWord = mState_now;

	return;
}

void SerialManipulator::UpdateManipulatorParam(void)
{
    for(int i=0; i < this->mDoF_Total; ++i)
    {
    	this->w[i] << serial_Kinematic_info[i].w_x, serial_Kinematic_info[i].w_y, serial_Kinematic_info[i].w_z;
		this->p[i] << serial_Kinematic_info[i].q_x, serial_Kinematic_info[i].q_y, serial_Kinematic_info[i].q_z;
		this->L[i] << serial_Kinematic_info[i].l_x, serial_Kinematic_info[i].l_y, serial_Kinematic_info[i].l_z;

		pKin->UpdateKinematicInfo( this->w[i], this->p[i], this->L[i], i );


    	Iner[i] << serial_Dynamic_info[i].Ixx_kgm2, serial_Dynamic_info[i].Ixy_kgm2, serial_Dynamic_info[i].Izx_kgm2,
    			serial_Dynamic_info[i].Ixy_kgm2, serial_Dynamic_info[i].Iyy_kgm2, serial_Dynamic_info[i].Iyz_kgm2,
				serial_Dynamic_info[i].Izx_kgm2, serial_Dynamic_info[i].Iyz_kgm2, serial_Dynamic_info[i].Izz_kgm2;


    	this->CoM[i] << serial_Dynamic_info[i].CoM_x, serial_Dynamic_info[i].CoM_y, serial_Dynamic_info[i].CoM_z;
    	mass[i] = serial_Dynamic_info[i].mass_kg;

    	pCoMKin->UpdateKinematicInfo( this->w[i], this->p[i], this->CoM[i], i );
    	pDyn->UpdateDynamicInfo( Iner[i], mass[i], CoM[i], i );

    }
}

void SerialManipulator::ENCtoRAD( int *_enc, double *_rad )
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_rad[i] = (double)(_enc[i])/(double)(serial_Motor_info[i].motor_harmonic*serial_Motor_info[i].enc_size)*(2.0*M_PI);
	}
	return;
}

void SerialManipulator::RADtoENC( int *_enc, double *_rad )
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_enc[i] = (int)(_rad[i]/(2*M_PI)*(serial_Motor_info[i].motor_harmonic*serial_Motor_info[i].enc_size));
	}
	return;
}

void SerialManipulator::VelocityConvert( int32_t *_enc_sec, double *_rad_sec )
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_rad_sec[i] = ((double)(_enc_sec[i])/(double)(serial_Motor_info[i].motor_harmonic * serial_Motor_info[i].enc_size))*(2.0*M_PI);
	}
	return;
}


void SerialManipulator::TorqueConvert(double *_pTorque, short *_pOutput, short &_MaxOutput)
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_pOutput[i] = (short)round(((_pTorque[i]/(double)serial_Motor_info[i].motor_harmonic)/serial_Motor_info[i].torque_const_Nm_A)/serial_Motor_info[i].max_current_A*1000.0*(100.0/80.0));

		if(_pOutput[i] <= -_MaxOutput)
			_pOutput[i] = -_MaxOutput;
		else if(_pOutput[i] >= _MaxOutput)
			_pOutput[i] = _MaxOutput;
	}
	return;
}

