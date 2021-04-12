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
}

Controller::Controller(std::shared_ptr<SerialManipulator> Manipulator)
{
	this->pManipulator = Manipulator;

	m_Jnum = this->pManipulator->GetTotalDoF();

	Kp.resize(m_Jnum);
	Kp.setConstant(KpBase);
	Kd.resize(m_Jnum);
	Kd.setConstant(KdBase);
    Ki.resize(m_Jnum);
    Ki.setConstant(KiBase);
	K_Hinf.resize(m_Jnum);
	K_Hinf.setConstant(HinfBase);

	KpTask.resize(6*this->pManipulator->GetTotalChain());
	KdTask.resize(6*this->pManipulator->GetTotalChain());
	KiTask.resize(6*this->pManipulator->GetTotalChain());

	e.setZero(m_Jnum);
	e_dev.setZero(m_Jnum);
	e_int.setZero(m_Jnum);
	e_int_sat.setZero(m_Jnum);

    eTask.setZero(6*this->pManipulator->GetTotalChain());
	edotTask.setZero(6*this->pManipulator->GetTotalChain());

	edotTmp.setZero(6*this->pManipulator->GetTotalChain(), 6*this->pManipulator->GetTotalChain());

	FrictionTorque.setZero(m_Jnum);

#if defined(__SIMULATION__)
    GainWeightFactor.resize(m_Jnum);
    GainWeightFactor.setConstant(7.0);

    dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);
    dq_old.setZero(16);

#else
    GainWeightFactor.setZero(m_Jnum);
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

	Kp = GainWeightFactor*KpBase;
	Kd = GainWeightFactor*KdBase;
	Ki = GainWeightFactor*KiBase;
	//K_Hinf = m_HinfBase;

	dq.resize(m_Jnum);
	dqdot.resize(m_Jnum);
	dqddot.resize(m_Jnum);
	dq_old.setZero(m_Jnum);
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
}

void Controller::SetPIDGain( VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki, VectorXd &_Kinf )
{
    K_Hinf = _Kinf;
    Kp = _Kinf.cwiseProduct(_Kp);
    Kd = _Kinf.cwiseProduct(_Kd);
    Ki = _Kinf.cwiseProduct(_Ki);
}

void Controller::GetPIDGain(double *_Kp, double *_Kd, double *_Hinf, int &_JointNum)
{
	_JointNum = this->m_Jnum;
	Map<VectorXd>(_Kp, this->m_Jnum) = Kp;
	Map<VectorXd>(_Kd, this->m_Jnum) = Kd;
	Map<VectorXd>(_Hinf, this->m_Jnum) = K_Hinf;
}

void Controller::GetPIDGain(VectorXd &_Kp, VectorXd &_Kd, VectorXd &_Ki)
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
}

void Controller::SetCLIKGain(const double &_Kp_Translation, const double &_Kp_Rotation)
{
    for(int i=0; i<pManipulator->GetTotalChain(); i++)
    {
        KpTask.segment(6*i,3) = _Kp_Rotation*Vector3d::Ones();
        KpTask.segment(6*i+3, 3) = _Kp_Translation*Vector3d::Ones();
    }
}

void Controller::SetTaskspaceGain(const VectorXd &_KpTask, const VectorXd &_KdTask)
{
    KpTask.setZero(12);
    KdTask.setZero(12);
    KpTask = _KpTask;
    KdTask = _KdTask;
}

void Controller::GetControllerStates(VectorXd &_dq, VectorXd &_dqdot, VectorXd &_ErrTask)
{
    _dq = dq;
    _dqdot = dqdot;
    _ErrTask = eTask;
}

void Controller::PDController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq)
{
    e = _dq - _q;
	e_dev = _dqdot - _qdot;

    _Toq.setZero(m_Jnum);
    _Toq.noalias() += Kp.cwiseProduct(e);
    _Toq.noalias() += Kd.cwiseProduct(e_dev);
}

void Controller::PDGravController( const VectorXd &_q, const VectorXd &_qdot, const VectorXd &_dq, const VectorXd &_dqdot, VectorXd &_Toq )
{
	pManipulator->pDyn->G_Matrix(G);

	e = _dq - _q;
	e_dev = _dqdot - _qdot;

	FrictionCompensator(_qdot, _dqdot);

    _Toq = G;
	_Toq.noalias() += Kp.cwiseProduct(e);
    _Toq.noalias() += Kd.cwiseProduct(e_dev);
    //_Toq.noalias() += FrictionTorque;
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
	dq_old = _dq;

	e = _dq - _q;
	e_dev = _dqdot - _qdot;
	//e_int += e*_dt*1e-6;
	//e_int_sat = tanh(e_int);

	FrictionCompensator(_qdot, _dqdot);

    VectorXd u0;
	u0.setZero(m_Jnum);
	u0.noalias() += _dqddot;
	u0.noalias() += Kd.cwiseProduct(e_dev);
	u0.noalias() += Kp.cwiseProduct(e);
    _Toq = G;
    _Toq.noalias() += M*u0;
    //_Toq.noalias() += FrictionTorque;
}

void Controller::TaskInvDynController(const VectorXd &_dx,
                                      const VectorXd &_dxdot,
                                      const VectorXd &_dxddot,
                                      const VectorXd &_q,
                                      const VectorXd &_qdot,
                                      VectorXd &_Toq,
                                      const double &_dt,
                                      const int mode)
{
    pManipulator->pDyn->MG_Mat_Joint(M, G);

    if( mode == 1 ) // Regulation
    {
        MatrixXd J;
        pManipulator->pKin->GetAnalyticJacobian(J);

        MatrixXd ScaledJT;
        pManipulator->pKin->GetScaledTransJacobian(ScaledJT);

        MatrixXd pInvJ;
        pManipulator->pKin->GetpinvJacobian(pInvJ);

        TaskError(_dx, _dxddot, _qdot, eTask, edotTask);

        VectorXd u0;
        u0.setZero(16);
        u0.noalias() += KpTask.cwiseProduct(eTask);
        u0.noalias() += -KdTask.cwiseProduct(J*_qdot);

        //MatrixXd mat_tmp = Matrix<double, 16, 16>::Identity();
        //mat_tmp.noalias() += -J.transpose()*J;
        //VectorXd q0dot;
        //alpha=5.0;
        //pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

        _Toq = G;
        _Toq.noalias() += ScaledJT*u0;
        //_Toq.noalias() += pInvJ*u0;
        //_Toq.noalias() += mat_tmp*q0dot;
    }
    else if( mode == 2 ) //tracking
    {
        MatrixXd ScaledJT;
        pManipulator->pKin->GetScaledTransJacobian(ScaledJT);

        MatrixXd Jdot;
        pManipulator->pKin->GetAnalyticJacobianDot(_qdot, Jdot);

        MatrixXd DpInvJ;
        pManipulator->pKin->GetDampedpInvJacobian(DpInvJ);

        TaskError(_dx, _dxddot, _qdot, eTask, edotTask);

        Matrix<double, 12, 1> uTask;
        uTask = _dxddot;
        uTask.noalias() += -Jdot*_qdot;
        uTask.noalias() += KdTask.cwiseProduct(edotTask);
        uTask.noalias() += KpTask.cwiseProduct(eTask);

        MatrixXd u0 = Matrix<double, 16,1>::Zero();
        //u0.noalias() += DpInvJ*uTask;
        u0.noalias() += ScaledJT*uTask;
        _Toq = G;
        _Toq.noalias() += M*u0;
    }
}

void Controller::TaskError(const VectorXd &_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot)
{
    _error_x.setZero(6*2);
    _error_xdot.setZero(6*2);

    int EndJoint[2] = {9, 16};
    SE3 aSE3;
    SO3 dSO3;
    Vector3d eOrient;

    for(int i=0; i<2; i++)
    {
        pManipulator->pKin->RollPitchYawtoSO3(_dx(6*i), _dx(6*i+1), _dx(6*i+2), dSO3);
        aSE3 = pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[i]);
        pManipulator->pKin->SO3toRollPitchYaw(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient);
        _error_x.segment(6*i,3) = eOrient;
        _error_x.segment(6*i+3,3) = _dx.segment(6*i+3, 3) - aSE3.block(0,3,3,1);
    }

    MatrixXd AnalyticJac;
    pManipulator->pKin->GetAnalyticJacobian(AnalyticJac);
    _error_xdot = _dxdot;
    _error_xdot.noalias() += -AnalyticJac*_qdot;
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

void Controller::CLIKTaskController( const VectorXd &_q,
                                     const VectorXd &_qdot,
                                     const VectorXd &_dx,
                                     const VectorXd &_dxdot,
                                     VectorXd &_Toq,
                                     const double &_dt,
                                     const int mode )
{
    TaskError(_dx, _dxdot, _qdot, eTask, edotTask);

    Vector_temp = _dxdot;
    Vector_temp.noalias() += KpTask.cwiseProduct(eTask);

	dq.setZero(16);
	dqdot.setZero(16);
	dqddot.setZero(16);

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJacobian);
    pManipulator->pKin->GetpinvJacobian(pInvJacobian);

    Matrix_temp = Eigen::MatrixXd::Identity(16,16);
    Matrix_temp += -pInvJacobian*AnalyticJacobian;

    if(mode == 1) // jacobian pseudoinverse
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

        dqdot.noalias() += pInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 2) // jacobian transpose
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

        dqdot.noalias() += AnalyticJacobian.transpose()*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode  == 3) // Damped jacobian pseudoinverse
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetDampedpInvJacobian(DampedpInvJacobian);

        dqdot.noalias() += DampedpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 4) // scaled jacobian transpose
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetScaledTransJacobian(ScaledTransJacobian);

        dqdot.noalias() += ScaledTransJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 5) // block jacobian pseudoinverse
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetBlockpInvJacobian(BlockpInvJacobian);

        dqdot.noalias() += BlockpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode  == 6) // weight damped jacobian pseudoinverse with task priority
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetWeightDampedpInvJacobian(Vector_temp, WdampedpInvJacobian);

        dqdot.noalias() += WdampedpInvJacobian*Vector_temp;
        //dqdot.noalias() += Matrix_temp*q0dot;
    }
    else
    {
        alpha = 2.0;
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetDampedpInvJacobian(DampedpInvJacobian);

        dqdot.noalias() += DampedpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }

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

	Kp = GainWeightFactor.cwiseProduct(Kp);
	Kd = GainWeightFactor.cwiseProduct(Kd);

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
