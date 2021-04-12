#include "SerialManipulator.h"

SerialManipulator::SerialManipulator()
{
	mChainMat.resize(2,16);
	mChainMat << 1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,
				1,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1;

	this->mDoF_Total = mChainMat.cols();
	this->mChain_Total = mChainMat.rows();

	pKin = std::make_unique<HYUMotionKinematics::PoEKinematics>(mChainMat);
	pDyn = std::make_unique<HYUMotionDynamics::Liedynamics>(mChainMat);
}

SerialManipulator::~SerialManipulator()
{

}

void SerialManipulator::StateMachine( const VectorXd _q, const VectorXd _qdot, VectorXd &_Target, uint16_t &_StateWord, uint16_t &_ControlWord )
{
	q = _q;
	qdot = _qdot;

	mState_now = _StateWord;

	if( _ControlWord == SYSTEM_BEGIN )
	{
		mState_now = SYSTEM_BEGIN;
	}
	else
	{
		q -= _Target;

		if((q.cwiseAbs().maxCoeff() <= 1.8*DEGtoRAD) && (qdot.cwiseAbs().maxCoeff() <= 0.1*DEGtoRAD))
			mState_now = TARGET_ACHIEVED;
		else
			mState_now = TARGET_MOVING;
	}

	mState_pre = mState_now;
	_StateWord = mState_now;
}

void SerialManipulator::UpdateManipulatorParam()
{
    for( int i=0; i < mDoF_Total; i++ )
    {
    	w[i] << serial_Kinematic_info[i].w_x, serial_Kinematic_info[i].w_y, serial_Kinematic_info[i].w_z;
		p[i] << serial_Kinematic_info[i].q_x, serial_Kinematic_info[i].q_y, serial_Kinematic_info[i].q_z;
		L[i] << serial_Kinematic_info[i].l_x, serial_Kinematic_info[i].l_y, serial_Kinematic_info[i].l_z;

		pKin->UpdateKinematicInfo( w[i], p[i], L[i], i );


    	Iner[i] << serial_Dynamic_info[i].Ixx_kgm2, serial_Dynamic_info[i].Ixy_kgm2, serial_Dynamic_info[i].Izx_kgm2,
    			serial_Dynamic_info[i].Ixy_kgm2, serial_Dynamic_info[i].Iyy_kgm2, serial_Dynamic_info[i].Iyz_kgm2,
				serial_Dynamic_info[i].Izx_kgm2, serial_Dynamic_info[i].Iyz_kgm2, serial_Dynamic_info[i].Izz_kgm2;

    	CoM[i] << serial_Dynamic_info[i].CoM_x, serial_Dynamic_info[i].CoM_y, serial_Dynamic_info[i].CoM_z;
    	mass[i] = serial_Dynamic_info[i].mass_kg;

    	pDyn->UpdateDynamicInfo( w[i], p[i], Iner[i], mass[i], CoM[i], i );
    }
}

void SerialManipulator::ENCtoRAD( int *_enc, VectorXd &_rad )
{
    _rad.setZero(mDoF_Total);
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_rad(i) = (double)(_enc[i] - serial_Motor_info[i].Offset)/(double)(serial_Motor_info[i].motor_harmonic*serial_Motor_info[i].enc_size)*(2.0*M_PI);
	}
}


void SerialManipulator::RADtoENC( int *_enc, VectorXd &_rad )
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_enc[i] = (int)(_rad(i)/(2*M_PI)*(serial_Motor_info[i].motor_harmonic*serial_Motor_info[i].enc_size) + serial_Motor_info[i].Offset);
	}
}

void SerialManipulator::VelocityConvert( int32_t *_enc_sec, VectorXd &_rad_sec )
{
    _rad_sec.setZero(mDoF_Total);
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_rad_sec(i) = ((double)(_enc_sec[i])/(double)(serial_Motor_info[i].motor_harmonic * serial_Motor_info[i].enc_size))*(2.0*M_PI);
	}
	return;
}


void SerialManipulator::TorqueConvert( VectorXd &_Torque, short *_pOutput, short &_MaxOutput )
{
	for(int i=0; i < this->mDoF_Total; i++)
	{
		_pOutput[i] = (short)round(((_Torque(i)/(double)serial_Motor_info[i].motor_harmonic)/serial_Motor_info[i].torque_const_Nm_A)/serial_Motor_info[i].max_current_A*1000.0*(100.0/70.0));

		if(_pOutput[i] <= -_MaxOutput)
			_pOutput[i] = -_MaxOutput;
		else if(_pOutput[i] >= _MaxOutput)
			_pOutput[i] = _MaxOutput;
	}
	return;
}

