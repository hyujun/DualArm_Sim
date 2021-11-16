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

	KpImp.setZero(6*this->pManipulator->GetTotalChain());
	KdImp.setZero(6*this->pManipulator->GetTotalChain());
	KpImpNull.setZero(m_Jnum);
	KdImpNull.setZero(m_Jnum);-

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
    GainWeightFactor.setConstant(15.0);

    dq.setZero(m_Jnum);
    dqdot.setZero(m_Jnum);
    dqddot.setZero(m_Jnum);
    dq_old.setZero(m_Jnum);

    KpImp.setConstant(12,1);
    KdImp.setConstant(12,0.2);
    KpImpNull.setConstant(16, 0.01);
    KdImpNull.setConstant(16,0.1);

#else
    GainWeightFactor.setZero(m_Jnum);
    GainWeightFactor.setConstant(15.0);

	Kp = GainWeightFactor*KpBase;
	Kd = GainWeightFactor*KdBase;
	Ki = GainWeightFactor*KiBase;

	dq.resize(m_Jnum);
	dqdot.resize(m_Jnum);
	dqddot.resize(m_Jnum);
	dq_old.setZero(m_Jnum);

    KpImp.setConstant(12,10.0);
    KdImp.setConstant(12,10.0);
    KpImpNull.setConstant(16,0.01);
    KdImpNull.setConstant(16,0.1);

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

void Controller::SetImpedanceGain( const VectorXd &_Kp_Imp, const VectorXd &_Kd_Imp, const VectorXd &_Kp_Imp_Null, const VectorXd &_Kd_Imp_Null, const VectorXd &_des_m )
{
    KpImp = _Kp_Imp;
    KdImp = _Kd_Imp;
    KpImpNull = _Kp_Imp_Null;
    KdImpNull = _Kd_Imp_Null;
    mass_shaped = _des_m;
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
#if !defined(__SIMULATION__)
    _Toq.noalias() += FrictionTorque;
#endif
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
	e_int += e*_dt;
    for(int i = 0;i<m_Jnum;i++)
        e_int(i) = tanh(e_int(i));

//	FrictionCompensator(_qdot, _dqdot);
    FrictionCompensator2( _dqdot);


    VectorXd u0;
	u0.setZero(m_Jnum);
	u0.noalias() += _dqddot;
	u0.noalias() += Kd.cwiseProduct(e_dev);
	u0.noalias() += Kp.cwiseProduct(e);
//    u0.noalias() += Ki.cwiseProduct(e_int);

    _Toq = G;
    _Toq.noalias() += M*u0;
//    _Toq.noalias() += FrictionTorque;

#if !defined(__SIMULATION__)
    //_Toq.noalias() += FrictionTorque;
#endif
}

void Controller::InvDynController2(const VectorXd &_q,
                                  const VectorXd &_qdot,
                                  const VectorXd &_dq,
                                  const VectorXd &_dqdot,
                                  const VectorXd &_dqddot,
                                  VectorXd &_Toq,
                                  VectorXd &_frictionToq,
                                  const double &_dt )
{
    pManipulator->pDyn->MG_Mat_Joint(M, G);
    //pManipulator->pDyn->G_Matrix(G);
    dq_old = _dq;

    e = _dq - _q;
    e_dev = _dqdot - _qdot;
    e_int += e*_dt*0.1;
//    for(int i = 0;i<m_Jnum;i++)
//        e_int(i) = tanh(e_int(i));

//	FrictionCompensator(_qdot, _dqdot);
    FrictionCompensator2( _dqdot);


    VectorXd u0;
    u0.setZero(m_Jnum);
    u0.noalias() += _dqddot;
    u0.noalias() += Kd.cwiseProduct(e_dev);
    u0.noalias() += Kp.cwiseProduct(e);
    u0.noalias() += Ki.cwiseProduct(e_int);

    _Toq = G;
    _Toq.noalias() += M*u0;
//    _Toq.noalias() += FrictionTorque;
    _frictionToq = FrictionTorque;

#if !defined(__SIMULATION__)
        //_Toq.noalias() += FrictionTorque;
#endif
    }

void Controller::TaskInvDynController( Cartesiand *_dx,
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

void Controller::TaskError( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot)
{
    _error_x.setZero(6*2);
    _error_xdot.setZero(6*2);

    int EndJoint[2] = {9, 16};
    SE3 aSE3;
    SO3 dSO3;
    Vector3d eOrient;
    double theta;
    for(int i=0; i<2; i++)
    {
        //dSO3 = _dx[i].r;
        aSE3 = pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[i]);
        //pManipulator->pKin->SO3toRollPitchYaw(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient);
        //pManipulator->pKin->LogSO3(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient,theta);
        //_error_x.segment(6*i,3) = eOrient;

        //Matrix3d SO3Tmp = aSE3.block(0,0,3,3).transpose()*dSO3;
        //Quaterniond eSO3;
        //eSO3 = SO3Tmp;

        Quaterniond q_d;
        q_d = _dx[i].r;
        Quaterniond q_a;
        q_a = pManipulator->pKin->GetForwardKinematicsSO3(EndJoint[i]);


        Vector3d e_orientation;
        Vector3d e_orientation2;

        Vector3d qd_vec;
        Vector3d qa_vec;

        qd_vec = q_d.vec();
        qa_vec = q_a.vec();

        double e_orientation1;
        e_orientation1 = q_a.w()*q_d.w()+qa_vec.transpose()*q_d.vec();
        e_orientation = q_d.w()*q_a.vec() - q_a.w()*q_d.vec() + SkewMatrix(qd_vec)*q_a.vec();
        //_error_x.segment(6*i,3) = eSO3.vec();
        _error_x.segment(6*i,3) = -e_orientation;
        _error_x.segment(6*i+3,3) = _dx[i].p - aSE3.block(0,3,3,1);
    }

    MatrixXd AnalyticJac;
    pManipulator->pKin->GetAnalyticJacobian(AnalyticJac);
    _error_xdot = _dxdot;
    _error_xdot.noalias() += -AnalyticJac*_qdot;
}

void Controller::TaskRelativeError( Cartesiand *_dx, const VectorXd &_dxdot, const VectorXd &_qdot, VectorXd &_error_x, VectorXd &_error_xdot )
{
    _error_x.setZero(6*2);
    _error_xdot.setZero(6*2);

    int EndJoint[2] = {9, 16};
    SE3 aSE3, aSE3_Rel;
    SO3 dSO3;
    MatrixXd AnalyticJac, RelativeJac;
    Vector3d eOrient;

    dSO3 = _dx[0].r;
    aSE3 = pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[0]);
    pManipulator->pKin->SO3toRollPitchYaw(aSE3.block(0,0,3,3).transpose()*dSO3, eOrient);
    _error_x.segment(0,3) = eOrient;
    _error_x.segment(3,3) = _dx[0].p - aSE3.block(0,3,3,1);

    dSO3 = _dx[1].r;
    aSE3_Rel.setZero();
    aSE3_Rel.noalias() += inverse_SE3(aSE3)*pManipulator->pKin->GetForwardKinematicsSE3(EndJoint[1]);
    pManipulator->pKin->SO3toRollPitchYaw(aSE3_Rel.block(0,0,3,3).transpose()*dSO3, eOrient);
    _error_x.segment(6,3) = eOrient;
    _error_x.segment(9,3) = _dx[1].p - aSE3_Rel.block(0,3,3,1);

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJac);
    _error_xdot.head(6) = _dxdot.head(6);
    _error_xdot.head(6).noalias() += -AnalyticJac.block(0,0,6,16)*_qdot;
    pManipulator->pKin->GetRelativeJacobian(RelativeJac);
    _error_xdot.tail(6) = _dxdot.tail(6);
    _error_xdot.tail(6).noalias() += -RelativeJac*_qdot;
}

void Controller::CLIKTaskController( const VectorXd &_q,
                                     const VectorXd &_qdot,
                                     Cartesiand *_dx,
                                     const VectorXd &_dxdot,
                                     const VectorXd &_sensor,
                                     VectorXd &_Toq,
                                     const double &_dt,
                                     const int mode )
{

    dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);

    pManipulator->pKin->GetAnalyticJacobian(AnalyticJacobian);
    pManipulator->pKin->GetpinvJacobian(pInvJacobian);
    alpha = 10.0;
    if( mode == 7 )
    {
        TaskRelativeError(_dx, _dxdot, _qdot, eTask, edotTask);
    }
    else
    {
        TaskError(_dx, _dxdot, _qdot, eTask, edotTask);
    }

    Vector_temp = _dxdot;
    Vector_temp.noalias() += KpTask.cwiseProduct(eTask);


    if(mode == 1) // jacobian pseudoinverse
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvJacobian*AnalyticJacobian;
        dqdot.noalias() += pInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 2) // jacobian transpose
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvJacobian*AnalyticJacobian;
        dqdot.noalias() += AnalyticJacobian.transpose()*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode  == 3) // Damped jacobian pseudoinverse
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetDampedpInvJacobian(DampedpInvJacobian);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvJacobian*AnalyticJacobian;
        dqdot.noalias() += DampedpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 4) // scaled jacobian transpose
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetScaledTransJacobian(ScaledTransJacobian);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvJacobian*AnalyticJacobian;
        dqdot.noalias() += ScaledTransJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 5) // block jacobian pseudoinverse
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetBlockpInvJacobian(BlockpInvJacobian);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -BlockpInvJacobian*AnalyticJacobian;
        dqdot.noalias() += BlockpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode  == 6) // weight damped jacobian pseudoinverse with task priority
    {
        MatrixXd weight;
        weight.setIdentity(16,16);
        //pManipulator->pDyn->M_Matrix(weight);
        //pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        VectorXd dx_tmp(12);
        Quaterniond q_tmp;
        q_tmp = _dx[0].r;
        dx_tmp.segment(0,3) = q_tmp.vec();
        dx_tmp.segment(3,3) = _dx[0].p;
        q_tmp = _dx[1].r;
        dx_tmp.segment(6,3) = q_tmp.vec();
        dx_tmp.segment(9,3) = _dx[1].p;

        pManipulator->pKin->GetWeightDampedpInvJacobian(dx_tmp, weight, WdampedpInvJacobian);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -WdampedpInvJacobian*AnalyticJacobian;
        dqdot.noalias() += WdampedpInvJacobian*Vector_temp;
        //dqdot.noalias() += Matrix_temp*q0dot;
    }
    else if(mode == 7) // relative jacobian
    {
        pManipulator->pKin->GetRelativeJacobian(RelJacobian);

        AJacwithRel = AnalyticJacobian;
        AJacwithRel.block(6,0,6,16) = RelJacobian;

        pManipulator->pKin->GetDampedpInvJacobian(AJacwithRel, dpInvRelJacobian);
        dqdot.noalias() += dpInvRelJacobian*Vector_temp;
    }
    else
    {
        pManipulator->pKin->Getq0dotWithMM(alpha, q0dot);
        pManipulator->pKin->GetDampedpInvJacobian(DampedpInvJacobian);

        dqdot.noalias() += DampedpInvJacobian*Vector_temp;
        dqdot.noalias() += Matrix_temp*q0dot;
    }

    dq = dq_old + dqdot * _dt;

    InvDynController(_q, _qdot, dq, dqdot, dqddot, _Toq, _dt);
}

void Controller::InertiaShaping( const VectorXd &_Mass, MatrixXd &_M_Shaped_inv )
{
    // ellipsoid volume = 4*M_PI*a*b*c/3
    const auto a=0.01; // radius(m) of x-axis
    const auto b=0.01; // radius(m) of y-axis
    const auto c=0.01; // radius(m) of z-axis

    _M_Shaped_inv.setZero(6*_Mass.size(),6*_Mass.size());
    _M_Shaped_inv(0,0) = 1.0/(_Mass(0)*(b*b + c*c)/5.0);
    _M_Shaped_inv(1,1) = 1.0/(_Mass(0)*(a*a + c*c)/5.0);
    _M_Shaped_inv(2,2) = 1.0/(_Mass(0)*(a*a + b*b)/5.0);
    _M_Shaped_inv.block(3,3,3,3).noalias() += Matrix<double, 3,3>::Identity()/_Mass(0);
    _M_Shaped_inv(6,6) = 1.0/(_Mass(1)*(b*b + c*c)/5.0);
    _M_Shaped_inv(7,7) = 1.0/(_Mass(1)*(a*a + c*c)/5.0);
    _M_Shaped_inv(8,8) = 1.0/(_Mass(1)*(a*a + b*b)/5.0);
    _M_Shaped_inv.block(9,9,3,3).noalias() += Matrix<double, 3,3>::Identity()/_Mass(1);
}

void Controller:: TaskImpedanceController(const VectorXd &_q, const VectorXd &_qdot, Cartesiand *_dx,
                                         const VectorXd &_dxdot, const VectorXd &_dxddot, const VectorXd &_sensor,
                                         VectorXd &_Toq, const int mode)
{
    MatrixXd pInvMat;
    pManipulator->pDyn->MG_Mat_Joint(M, G);
    pManipulator->pKin->GetAnalyticJacobian(AnalyticJacobian);
    pManipulator->pKin->GetAnalyticJacobianDot(_qdot, AnalyticJacobianDot);

    dqN.setZero(16);
    dqdotN.setZero(16);

    alpha = 7.5;

    if(mode == 1) // Mx = Mx_desired
    {
        VectorXd u01 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u02 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u04 = VectorXd::Zero(AnalyticJacobian.cols());

        u01 = _dxddot;
        u01.noalias() += -AnalyticJacobianDot*_qdot;

        TaskError(_dx, _dxdot, _qdot, eTask, edotTask);

        u02.noalias() += KdImp.cwiseProduct(edotTask);
        u02.noalias() += KpImp.cwiseProduct(eTask);

        //dqN = 0.5*(pManipulator->pKin->qLimit_High - pManipulator->pKin->qLimit_Low);
        pManipulator->pKin->Getq0dotWithMM(alpha, dqdotN);
        //dqdotN.noalias() += -0.5*(pManipulator->pKin->qLimit_Low - _q).cwiseInverse();
        //dqdotN.noalias() += 0.5*(_q - pManipulator->pKin->qLimit_High).cwiseInverse();
        //u04.noalias() += KpImpNull.cwiseProduct(dqN - _q);
        u04.noalias() += KdImpNull.cwiseProduct(dqdotN - _qdot);

        MatrixXd weight;
        //weight.setIdentity(16,16);
        weight = M;

        VectorXd dx_tmp(12);
        Quaterniond q_tmp;
        q_tmp = _dx[0].r;
        dx_tmp.segment(0,3) = q_tmp.vec();
        dx_tmp.segment(3,3) = _dx[0].p;
        q_tmp = _dx[1].r;
        dx_tmp.segment(6,3) = q_tmp.vec();
        dx_tmp.segment(9,3) = _dx[1].p;

        pManipulator->pKin->GetWeightDampedpInvJacobian(dx_tmp, weight, AnalyticJacobian, pInvMat);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvMat*AnalyticJacobian;

        _Toq = G;
        _Toq.noalias() += M*(pInvMat*u01);
        _Toq.noalias() += AnalyticJacobian.transpose()*u02;
        _Toq.noalias() += Matrix_temp*u04;
    }
    else if(mode == 2) // Mx != Mx_desired
    {
        MatrixXd MxdInv;
        InertiaShaping(mass_shaped, MxdInv);

        pManipulator->pKin->GetDampedpInvJacobian(pInvMat);
        pManipulator->pDyn->M_Mat_Task(Mx, pInvMat);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvMat*AnalyticJacobian;

        VectorXd u01 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u02 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u03 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u04 = VectorXd::Zero(AnalyticJacobian.cols());

        u01 = _dxddot ;
        u01.noalias() += -AnalyticJacobianDot*_qdot;

        TaskError(_dx, _dxdot, _qdot, eTask, edotTask);
        u02.noalias() += KdImp.cwiseProduct(edotTask);
        u02.noalias() += KpImp.cwiseProduct(eTask);

        MatrixXd M_Shaped = MatrixXd::Zero(AnalyticJacobian.rows(),AnalyticJacobian.rows());
        M_Shaped.noalias() += Mx*MxdInv;
        u03.noalias() += M_Shaped*u02;
        u03.noalias() += (M_Shaped - MatrixXd::Identity(AnalyticJacobian.rows(),AnalyticJacobian.rows()))*_sensor;

        dqN = 0.5*(pManipulator->pKin->qLimit_High - pManipulator->pKin->qLimit_Low);
        pManipulator->pKin->Getq0dotWithMM(alpha, dqdotN);
        dqdotN.noalias() += 0.5*(pManipulator->pKin->qLimit_Low - _q).cwiseInverse();
        dqdotN.noalias() += -0.5*(_q - pManipulator->pKin->qLimit_High).cwiseInverse();
        u04.noalias() += KpImpNull.cwiseProduct(dqN - _q);
        u04.noalias() += KdImpNull.cwiseProduct(dqdotN - _qdot);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvMat*AnalyticJacobian;

        _Toq = G;
        _Toq.noalias() += M*(pInvMat*u01);
        _Toq.noalias() += AnalyticJacobian.transpose()*u03;
        _Toq.noalias() += Matrix_temp*u04;
    }
    else if(mode == 3) // relative jacobian
    {
        pManipulator->pKin->GetRelativeJacobian(RelativeJacobian);
        pManipulator->pKin->GetRelativeJacobianDot(_qdot, RelativeJacobianDot);
        AnalyticJacobian.block(6,0,6,16) = RelativeJacobian;
        AnalyticJacobianDot.block(6,0,6,16) = RelativeJacobianDot;

        VectorXd u01 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u02 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u04 = VectorXd::Zero(AnalyticJacobian.cols());

        u01 = _dxddot;
        u01.noalias() += -AnalyticJacobianDot*_qdot;

        TaskRelativeError(_dx, _dxdot, _qdot, eTask, edotTask);
        u02.noalias() += KdImp.cwiseProduct(edotTask);
        u02.noalias() += KpImp.cwiseProduct(eTask);

        //dqN = 0.5*(pManipulator->pKin->qLimit_High - pManipulator->pKin->qLimit_Low);
        pManipulator->pKin->Getq0dotWithMM(alpha, dqdotN);
        //pManipulator->pKin->Getq0dotWithMM_Relative(alpha, AnalyticJacobian, dqdotN);
        //dqdotN.noalias() += 0.5*(pManipulator->pKin->qLimit_Low - _q).cwiseInverse();
        //dqdotN.noalias() += -0.5*(_q - pManipulator->pKin->qLimit_High).cwiseInverse();
        //u04.noalias() += KpImpNull.cwiseProduct(dqN - _q);
        u04.noalias() += KdImpNull.cwiseProduct(dqdotN - _qdot);

        MatrixXd weight;
        //weight.setIdentity(16,16);
        weight = M;

        VectorXd dx_tmp(12);
        Quaterniond q_tmp;
        q_tmp = _dx[0].r;
        dx_tmp.segment(0,3) = q_tmp.vec();
        dx_tmp.segment(3,3) = _dx[0].p;
        q_tmp = _dx[1].r;
        dx_tmp.segment(6,3) = q_tmp.vec();
        dx_tmp.segment(9,3) = _dx[1].p;

        pManipulator->pKin->GetWeightDampedpInvJacobian(dx_tmp, weight, AnalyticJacobian, pInvMat);

        //pManipulator->pKin->GetDampedpInvBlockJacobian(AnalyticJacobian, pInvMat);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvMat*AnalyticJacobian;

        _Toq = G;
        _Toq.noalias() += M*(pInvMat*u01);
        _Toq.noalias() += AnalyticJacobian.transpose()*u02;
        _Toq.noalias() += Matrix_temp*u04;
    }
    else
    {
        pManipulator->pKin->GetDampedpInvJacobian(pInvMat);

        Matrix_temp = Eigen::MatrixXd::Identity(16,16);
        Matrix_temp += -pInvMat*AnalyticJacobian;

        VectorXd u01 = VectorXd::Zero(AnalyticJacobian.rows());
        VectorXd u02 = VectorXd::Zero(AnalyticJacobian.rows());

        u01 = _dxddot;
        u01.noalias() += -AnalyticJacobianDot*_qdot;

        TaskError(_dx, _dxdot, _qdot, eTask, edotTask);
        u02.noalias() += KdImp.cwiseProduct(edotTask);
        u02.noalias() += KpImp.cwiseProduct(eTask);

        _Toq = G;
        _Toq.noalias() += M*(pInvMat*u01);
        _Toq.noalias() += AnalyticJacobian.transpose()*u02;
    }
}
    double SignFunction(double value)
    {
        if (value<0.0f)
        {
            return -1.0f;
        }
        else if(value>0.0f)
        {
            return 1.0f;
        }
        else return 0.0f;
        return 0.0f;
    }

    void Controller::FrictionCompensator2( const VectorXd &_dqdot)
    {
        FrictionTorque.setZero(m_Jnum);

        FrictionTorque(0) = 18.3f*0.8*SignFunction(_dqdot(0));
        FrictionTorque(1) = 25.6f*0.5*SignFunction(_dqdot(1));
        FrictionTorque(2) = 6.8f*SignFunction(_dqdot(2));
        FrictionTorque(3) = 4.3f*SignFunction(_dqdot(3));
        FrictionTorque(4) = 7.2f*SignFunction(_dqdot(4));
        FrictionTorque(5) = 4.08f*SignFunction(_dqdot(5));
        FrictionTorque(6) = 4.24f*SignFunction(_dqdot(6));
        FrictionTorque(7) = 3.04f*SignFunction(_dqdot(7));
        FrictionTorque(8) = 2.56f*SignFunction(_dqdot(8));
        FrictionTorque(9) = 9.2f*SignFunction(_dqdot(9));
        FrictionTorque(10) = 5.2f*SignFunction(_dqdot(10));
        FrictionTorque(11) = 7.0f*SignFunction(_dqdot(11));
        FrictionTorque(12) = 4.4*SignFunction(_dqdot(12));
        FrictionTorque(13) = 2.4*SignFunction(_dqdot(13));
        FrictionTorque(14) = 3.6f*SignFunction(_dqdot(14));
        FrictionTorque(15) = 2.24f*SignFunction(_dqdot(15));
    }

void Controller::FrictionIdentification( const VectorXd &_q, const VectorXd &_qdot, VectorXd &_dq, VectorXd &_dqdot, VectorXd &_dqddot, VectorXd &_Toq, const double &gt )
{
    GainWeightFactor(4.0);

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

	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G + FrictionTorque;
	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;

	VectorXd u0;
    u0.setZero(m_Jnum);
    u0.noalias() += _dqddot;
    u0.noalias() += Kd.cwiseProduct(e_dev);
    u0.noalias() += Kp.cwiseProduct(e);

    _Toq.setZero(m_Jnum);
    _Toq = G;
    _Toq.noalias() += M*u0;
    _Toq.noalias() += FrictionTorque;
}

void Controller::FrictionCompensator( const VectorXd &_qdot, const VectorXd &_dqdot )
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
