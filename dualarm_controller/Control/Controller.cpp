/*
 * Controller.cpp
 *
 *  Created on: 2019. 5. 15.
 *      Author: Junho Park
 */

#include "Controller.h"

#include <utility>

namespace HYUControl {

Controller::Controller():m_Jnum(6)
{
	this->pManipulator = nullptr;
    m_KpBase = KpBase;
    m_KdBase = KdBase;
    m_KiBase = KiBase;
    m_HinfBase = HinfBase;
}

Controller::Controller(std::shared_ptr<SerialManipulator> pManipulator)
{
	this->pManipulator = std::move(pManipulator);

	m_Jnum = this->pManipulator->GetTotalDoF();
	m_KpBase = KpBase;
	m_KdBase = KdBase;
	m_KiBase = KiBase;
	m_HinfBase = HinfBase;

	Kp.resize(m_Jnum);
	Kp.setConstant(m_KpBase);
	Kd.resize(m_Jnum);
	Kd.setConstant(m_KdBase);
    Ki.resize(m_Jnum);
    Ki.setConstant(m_KiBase);
	K_Hinf.resize(m_Jnum);
	K_Hinf.setConstant(m_HinfBase);

	KpTask.resize(6*this->pManipulator->pKin->GetNumChain());
	KdTask.resize(6*this->pManipulator->pKin->GetNumChain());
	KiTask.resize(6*this->pManipulator->pKin->GetNumChain());

	e.resize(m_Jnum);
	e.setZero();

	e_dev.resize(m_Jnum);
	e_dev.setZero();

	e_int.resize(m_Jnum);
	e_int_sat.resize(m_Jnum);

	edotTask.resize(6*this->pManipulator->GetTotalChain());
	eTask.resize(6*this->pManipulator->GetTotalChain());

	edotTmp.resize(6*this->pManipulator->GetTotalChain(), 6*this->pManipulator->GetTotalChain());

	FrictionTorque.resize(m_Jnum);
	FrictionTorque.setZero();

	GainWeightFactor.resize(m_Jnum);
#if defined(__SIMULATION__)
    GainWeightFactor.resize(m_Jnum);
    GainWeightFactor.setConstant(7.0);

    Kp = GainWeightFactor*m_KpBase;
    Kd = GainWeightFactor*m_KdBase;
    Ki = GainWeightFactor*m_KiBase;
    K_Hinf.setConstant(m_HinfBase);

    dq.resize(m_Jnum);
    dqdot.resize(m_Jnum);
    dqddot.resize(m_Jnum);

    KpTask(0,0) = 10.0;
    KpTask(1,1) = 10.0;
    KpTask(2,2) = 10.0;

    KpTask(3,3) = 10.0;
    KpTask(4,4) = 10.0;
    KpTask(5,5) = 10.0;

    KpTask(6,6) = 10.0;
    KpTask(7,7) = 10.0;
    KpTask(8,8) = 10.0;

    KpTask(9,9) = 10.0;
    KpTask(10,10) = 10.0;
    KpTask(11,11) = 10.0;

    KdTask(0,0) = 0.0001;
    KdTask(1,1) = 0.0001;
    KdTask(2,2) = 0.0001;

    KdTask(3,3) = 0.001;
    KdTask(4,4) = 0.001;
    KdTask(5,5) = 0.001;

    KdTask(6,6) = 0.0001;
    KdTask(7,7) = 0.0001;
    KdTask(8,8) = 0.0001;

    KdTask(9,9) = 0.001;
    KdTask(10,10) = 0.001;
    KdTask(11,11) = 0.001;
#else
	GainWeightFactor(0) = 20.0;
	GainWeightFactor(1) = 23.0;

	GainWeightFactor(2) = 11.5;
	GainWeightFactor(3) = 20.5; //8
	GainWeightFactor(4) = 11.0;
	GainWeightFactor(5) = 9.0;
	GainWeightFactor(6) = 5.5;
	GainWeightFactor(7) = 5.5;
	GainWeightFactor(8) = 5.5;

	GainWeightFactor(9) = 11.5;
	GainWeightFactor(10) = 11.5; //8
	GainWeightFactor(11) = 11.5;
	GainWeightFactor(12) = 9.0;
	GainWeightFactor(13) = 5.5;
	GainWeightFactor(14) = 5.5;
	GainWeightFactor(15) = 5.5;

	Kp = GainWeightFactor*m_KpBase;
	Kd = GainWeightFactor*m_KdBase;
	Ki = GainWeightFactor*m_KiBase;
	//K_Hinf = m_HinfBase;

	dq.resize(m_Jnum);
	dqdot.resize(m_Jnum);
	dqddot.resize(m_Jnum);

	KpTask(0) = 0.00001;
	KpTask(1) = 0.00001;
	KpTask(2) = 0.00001;

	KpTask(3) = 0.0001;
	KpTask(4) = 0.0001;
	KpTask(5) = 0.0001;

	KpTask.tail(6) = KpTask.head(6);
#endif

}

Controller::~Controller() {

}

void Controller::ClearError(void)
{
	e.setZero();
	e_dev.setZero();
	e_int.setZero();
	return;
}

void Controller::SetPIDGain(double &_Kp, double &_Kd, double &_Hinf, int &_JointNum)
{
	Kp(_JointNum-1) = _Kp;
	Kd(_JointNum-1) = _Kd;
	K_Hinf(_JointNum-1) = _Hinf;
	return;
}

void Controller::SetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki, VectorXd &_Kinf)
{
    K_Hinf = _Kinf;
    Kp = _Kinf.asDiagonal()*_Kp;
    Kd = _Kinf.asDiagonal()*_Kd;
    Ki = _Kinf.asDiagonal()*_Ki;

    return;
}

void Controller::GetPIDGain(double *_Kp, double *_Kd, double *_Hinf, int &_JointNum)
{
	_JointNum = this->m_Jnum;
	Map<VectorXd>(_Kp, this->m_Jnum) = Kp;
	Map<VectorXd>(_Kd, this->m_Jnum) = Kd;
	Map<VectorXd>(_Hinf, this->m_Jnum) = K_Hinf;
	return;
}

void Controller::GetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki)
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    return;
}

void Controller::PDController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq)
{
    e = _dq - _q;
	e_dev = _dqdot - _qdot;

    _Toq.setZero(m_Jnum);
    _Toq.noalias() += e.cwiseProduct(Kp) + e_dev.cwiseProduct(Kd);
}

void Controller::PDGravController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq )
{
	pManipulator->pDyn->G_Matrix(G);

	e = _dq - _q;
	e_dev = _dqdot - _qdot;

	FrictionCompensator(_qdot, _dqdot);

    _Toq.setZero();
	_Toq.noalias() += Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;
    //_Toq.noalias() += G + FrictionTorque;
}

void Controller::InvDynController(const VectorXd &_q,
                                  const VectorXd &_qdot,
                                  const VectorXd &_dq,
                                  const VectorXd &_dqdot,
                                  const VectorXd &_dqddot,
                                  VectorXd &_Toq,
                                  const double &_dt )
{
	pManipulator->pDyn->MG_Mat_Joint(M, G);
	//pManipulator->pDyn->G_Matrix(G);
    VectorXd u0(m_Jnum);
	e = _dq - _q;
	e_dev = _dqdot - _qdot;
	//e_int += e*_dt*1e-6;
	//e_int_sat = tanh(e_int);

	FrictionCompensator(_qdot, _dqdot);

	if(_Toq.size() != m_Jnum)
        _Toq.setZero(m_Jnum);
	else
	    _Toq.setZero();

	u0.setZero();
	u0.noalias() += _dqddot;
	u0.noalias() += Kd.cwiseProduct(e_dev);
	u0.noalias() += Kp.cwiseProduct(e);
    _Toq.noalias() += G;
    _Toq.noalias() += M*u0;
    //_Toq.noalias() += FrictionTorque;
}

void Controller::TaskInvDynController(const VectorXd &_dx,
                                      const VectorXd &_dxdot,
                                      const VectorXd &_q,
                                      const VectorXd &_qdot,
                                      VectorXd &_Toq,
                                      const double &_dt)
{
    pManipulator->pDyn->MG_Mat_Joint(M, G);

    MatrixXd AJacobian;
    MatrixXd pInvJac;
    MatrixXd eye = MatrixXd::Identity(m_Jnum,m_Jnum);
    VectorXd q0dot;

    pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
    pManipulator->pKin->GetAnalyticJacobian(AJacobian);

    _Toq.setZero(m_Jnum);
    _Toq.noalias() += AJacobian.transpose()*(-KpTask*_dx - KdTask*AJacobian*_qdot) + (eye - AJacobian.transpose()*AJacobian)*q0dot + G;
}

void Controller::TaskError(const VectorXd &_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot)
{
    _error_x.setZero(6*2);
    _error_xdot.setZero(6*2);

    int EndJoint[2] = {9, 16};
    SE3 aSE3;
    SO3 dSO3;
    MatrixXd AnalyticJac;
    Vector3d eOrient;

    for(int i=0; i<2; i++)
    {
        pManipulator->pKin->RollPitchYawtoSO3(_dx(6*i), _dx(6*i+1), _dx(6*i+2), dSO3);
        aSE3 = pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[i]);
        pManipulator->pKin->SO3toRollPitchYaw(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient);
        _error_x.segment(6*i,3) = eOrient;
        _error_x.segment(6*i+3,3) = _dx.segment(6*i+3, 3) - aSE3.block(0,3,3,1);
    }

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJac);
    _error_xdot = _dxdot - AnalyticJac*_qdot;
}

void Controller::TaskRelativeError(const VectorXd &_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot)
{
    _error_x.setZero(6*2);
    _error_xdot.setZero(6*2);

    int EndJoint[2] = {9, 16};
    SE3 aSE3, aSE3_Rel;
    SO3 dSO3;
    MatrixXd AnalyticJac, RelativeJac;
    Vector3d eOrient;

    pManipulator->pKin->RollPitchYawtoSO3(_dx(0), _dx(1), _dx(2), dSO3);
    aSE3 = pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[0]);
    pManipulator->pKin->SO3toRollPitchYaw(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient);
    _error_x.segment(0,3) = eOrient;
    _error_x.segment(3,3) = _dx.segment(3, 3) - aSE3.block(0,3,3,1);

    pManipulator->pKin->RollPitchYawtoSO3(_dx(6), _dx(7), _dx(8), dSO3);
    aSE3_Rel.setZero();
    aSE3_Rel.noalias() += inverse_SE3(aSE3)*pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[1]);
    pManipulator->pKin->SO3toRollPitchYaw(aSE3_Rel.block(0,0,3,3).transpose()*dSO3, eOrient);
    _error_x.segment(6,3) = eOrient;
    _error_x.segment(9,3) = _dx.segment(9, 3) - aSE3_Rel.block(0,3,3,1);

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJac);
    _error_xdot.head(6) = _dxdot.head(6);
    _error_xdot.head(6).noalias() += -AnalyticJac.block(0,0,6,16)*_qdot;
    pManipulator->pKin->GetRelativeJacobian(RelativeJac);
    _error_xdot.tail(6) = _dxdot.tail(6);
    _error_xdot.tail(6).noalias() += -RelativeJac*_qdot;
}

void Controller::CLIKTaskController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dx, const VectorXd &_dxdot, VectorXd &_Toq, const double &_dt )
{
    eTask.setZero(6*pManipulator->GetTotalChain());
	edotTask.setZero(6*pManipulator->GetTotalChain());

    TaskError(_dx, _dxdot, _qdot, eTask, edotTask);

	dq.setZero();
	dqdot.setZero();
	dqddot.setZero();

	VectorXd q0dot;

    pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

    //pManipulator->pKin->GetpinvJacobian(pInvJacobian);
    //dqdot = pInvJacobian * (_dxdot + KpTask.cwiseProduct(eTask)) + (Eigen::MatrixXd::Identity(16,16) - AnalyticJacobian.transpose()*AnalyticJacobian)*q0dot;

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJacobian);
    dqdot = AnalyticJacobian.transpose() * (_dxdot + KpTask.cwiseProduct(eTask)) + (Eigen::MatrixXd::Identity(16,16) - AnalyticJacobian.transpose()*AnalyticJacobian)*q0dot;

    //pManipulator->pKin->GetScaledTransJacobian(ScaledTransJacobian);
    //dqdot = ScaledTransJacobian * (_dxdot + KpTask.cwiseProduct(eTask)) + (Eigen::MatrixXd::Identity(16,16) - AnalyticJacobian.transpose()*AnalyticJacobian)*q0dot;

    dq = dq_old + dqdot * _dt;
    dq_old = dq;

    InvDynController(_q, _qdot, dq, dqdot, dqddot, _Toq, _dt);
}

void Controller::FrictionIdentification( const VectorXd &_q, const VectorXd &_qdot, VectorXd &_dq, VectorXd &_dqdot, VectorXd &_dqddot, VectorXd &_Toq, const double &gt )
{
	GainWeightFactor(0) = 7.0;
	GainWeightFactor(1) = 4.0;

	GainWeightFactor(2) = 2.0;
	GainWeightFactor(3) = 1.5; //8
	GainWeightFactor(4) = 3.0;
	GainWeightFactor(5) = 1.0;
	GainWeightFactor(6) = 0.8;
	GainWeightFactor(7) = 1.0;

	GainWeightFactor(8) = 2.0;
	GainWeightFactor(9) = 1.5; //8
	GainWeightFactor(10) = 3.0;
	GainWeightFactor(11) = 1.0;
	GainWeightFactor(12) = 0.8;
	GainWeightFactor(13) = 1.0;

	Kp = GainWeightFactor*m_KpBase;
	Kd = GainWeightFactor*m_KdBase;

	_dq.setZero();
	_dqdot.setZero();
	_dqddot.setZero();

	int testjoint = 4;

	double T, omega, amp;

	if(InitTime == 0)
	{
		InitTime = gt;
	}
	else
	{
		switch(testjoint)
		{
		case 0:
			T = 29.3;
			omega = 2.0*M_PI/T;
			amp = 70;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));
			break;

		case 1:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.8481) - 30*M_PI/180.0;
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.8481);
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.8481);
			break;

		case 2:
			T = 20.9;
			omega = 2.0*M_PI/T;
			amp = 50;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.6435) - 30*M_PI/180.0;
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.6435);
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.6435);

			_dq(testjoint+6) = -_dq(testjoint);
			_dqdot(testjoint+6) = -_dqdot(testjoint);
			_dqddot(testjoint+6) = -_dqddot(testjoint);
			break;

		case 3:
			T = 30.0;
			omega = 2.0*M_PI/T;
			amp = 50;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.4115) - 20*M_PI/180.0;
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.4115);
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.4115);

			_dq(testjoint+6) = -_dq(testjoint);
			_dqdot(testjoint+6) = -_dqdot(testjoint);
			_dqddot(testjoint+6) = -_dqddot(testjoint);
			break;

		case 4:
			T = 18.8;
			omega = 2.0*M_PI/T;
			amp = 45;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.729) - 30*M_PI/180.0;
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.729);
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.729);

			_dq(testjoint+6) = -_dq(testjoint);
			_dqdot(testjoint+6) = -_dqdot(testjoint);
			_dqddot(testjoint+6) = -_dqddot(testjoint);
			break;

		case 5:
			T = 29.3;
			omega = 2.0*M_PI/T;
			amp = 70.0;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			_dq(testjoint+6) = _dq(testjoint);
			_dqdot(testjoint+6) = _dqdot(testjoint);
			_dqddot(testjoint+6) = _dqddot(testjoint);
			break;

		case 6:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40.0;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			_dq(testjoint+6) = _dq(testjoint);
			_dqdot(testjoint+6) = _dqdot(testjoint);
			_dqddot(testjoint+6) = _dqddot(testjoint);
			break;

		case 7:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40.0;

			_dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			_dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			_dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			_dq(testjoint+6) = _dq(testjoint);
			_dqdot(testjoint+6) = _dqdot(testjoint);
			_dqddot(testjoint+6) = _dqddot(testjoint);
			break;

		default:
			_dq.setZero();
			_dqdot.setZero();
			_dqddot.setZero();
			break;
		}
	}

    pManipulator->pDyn->MG_Mat_Joint(M, G);
	e = _dq - _q;
	e_dev = _dqdot - _qdot;

	FrictionCompensator(_qdot, _dqdot);

	_Toq.setZero(m_Jnum);
	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G + FrictionTorque;
	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;
    _Toq.noalias() += Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;
}

void Controller::FrictionCompensator( const VectorXd &_qdot,  const VectorXd &_dqdot )
{
	FrictionTorque.setZero(m_Jnum);

	for(int i=0; i < this->m_Jnum; i++)
	{
		FrictionTorque(i) = frictiontanh[i].a*(tanh(frictiontanh[i].b*_dqdot(i)) - tanh(frictiontanh[i].c*_dqdot(i))) + frictiontanh[i].d*tanh(frictiontanh[i].e*_dqdot(i)) + frictiontanh[i].f*_dqdot(i);
	}
}

void Controller::OutputSaturation(double *pInput , double &_MaxInput)
{
	for(int i=0; i<m_Jnum; ++i)
	{
		if(pInput[i] <= -_MaxInput)
		{
			pInput[i] = -_MaxInput;
		}
		else if(pInput[i] >= _MaxInput)
		{
			pInput[i] = _MaxInput;
		}
	}
}


} /* namespace HYUCtrl */
