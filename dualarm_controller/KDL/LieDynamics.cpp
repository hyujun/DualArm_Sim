#include "LieDynamics.h"


// how to use
// 1. Update_info
// 2. Prepare_Dynamics
// 3. Get M_Matrix, C_Matrix, G_Matrix

namespace HYUMotionDynamics{

    Liedynamics::Liedynamics():isFirstRun(0)
    {
        pLink = NULL;
        pCoM = NULL;

        m_NumChain = 1;
        m_DoF = 6;

        this->Iner_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->Gamma_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->L_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->ad_Aqd.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->ad_V.conservativeResize(6*this->m_DoF, 6*this->m_DoF);

        this->A_mat.conservativeResize(6*this->m_DoF, this->m_DoF);
        this->LA_mat.conservativeResize(6*this->m_DoF, this->m_DoF);

        this->V.conservativeResize(6*this->m_DoF);
        this->VdotBase.conservativeResize(6*this->m_DoF);

        grav.resize(6);
        grav << 0, 0, 0, 0, 0, 9.8;
    }

    Liedynamics::Liedynamics( const MatrixXi &_ChainMatrix, HYUMotionKinematics::PoEKinematics &_PoEKin, HYUMotionKinematics::PoEKinematics &_CoMKin ):isFirstRun(0)
    {
        pLink = &_PoEKin;
        pCoM = &_CoMKin;

        this->ChainMatrix = _ChainMatrix;

        this->m_NumChain = ChainMatrix.rows();
        this->m_DoF = ChainMatrix.cols();

        this->Iner_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->Gamma_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->L_mat.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->ad_Aqd.conservativeResize(6*this->m_DoF, 6*this->m_DoF);
        this->ad_V.conservativeResize(6*this->m_DoF, 6*this->m_DoF);

        this->A_mat.conservativeResize(6*this->m_DoF, this->m_DoF);
        this->LA_mat.conservativeResize(6*this->m_DoF, this->m_DoF);

        this->V.conservativeResize(6*this->m_DoF);
        this->VdotBase.conservativeResize(6*this->m_DoF);

        grav.resize(6);
        grav.setZero();
        grav << 0, 0, 0, 0, 0, 9.8;

        Eigen::initParallel();
    }

    Liedynamics::~Liedynamics()
    {

    }

    void Liedynamics::UpdateDynamicInfo( Matrix3d _Inertia, double _Mass, Vector3d _CoM, int _LinkNum )
    {
        GeneralizedInertia(_Inertia, _Mass, this->GIner[_LinkNum]);
        this->A[_LinkNum] = AdjointMatrix(inverse_SE3(pCoM->GetMMat(_LinkNum)))*pLink->GetTwist(_LinkNum);
    }

    void Liedynamics::GeneralizedInertia(const Matrix3d &_Inertia, const double &_Mass, Matrix6d &GIner)
    {
        GIner.setZero();
        GIner.block<3, 3>(0, 0) = _Inertia; /**If the Coordinate is changed, then THE INERTIA HAVE TO CHANGE w.r.t. the coordinate*/
        GIner.block<3, 3>(3, 3).noalias() += _Mass*MatrixXd::Identity(3,3);
        return;
    }


    void Liedynamics::Inertia_Link( void )
    {
        this->Iner_mat.setZero();
        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->Iner_mat.block<6, 6>(6 * i, 6 * i) = this->GIner[i];
        }
        return;
    }

    void Liedynamics::Gamma_Link( void )
    {
        this->Gamma_mat.setZero();
        for (int i = 1; i < this->m_DoF; ++i)
        {
            this->Gamma_mat.block<6, 6>(6 * i, 6 * (i - 1)) = LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pCoM->GetTMat(i, i+1)));
            //LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pPoEKinematics->GetTMat(i, i + 1)), this->Gamma_mat.block<6, 6>(6 * i, 6 * (i - 1)));
        }
        return;
    }

    void Liedynamics::L_link( )
    {
        this->L_mat.setIdentity();
        for(int k=0; k < this->m_NumChain; k++)
        {
            for (int i = 0; i < this->m_DoF; ++i)
            {
                if(ChainMatrix(k,i) == 1)
                {
                    for(int j=i+1; j< this->m_DoF; ++j)
                    {
                        if(ChainMatrix(k,j) == 1)
                        {
                            this->L_mat.block(6*j, 6*i, 6, 6) = LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pCoM->GetTMat(i+1,j+1)));
                            //LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pPoEKinematics->GetTMat(i + 1, j + 1)), this->L_mat.block(6 * j, 6 * i, 6, 6));
                        }
                    }
                }
            }
        }
    }

    void Liedynamics::A_Link( void )
    {
        this->A_mat.setZero();
        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->A_mat.block<6, 1>(6 * i,  i) = A[i];
        }
        return;
    }

    void Liedynamics::ad_Aqdot_Link( VectorXd _qdot )
    {
        this->ad_Aqd.setZero();
        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->ad_Aqd.block<6, 6>(6 * i, 6 * i) = LieOperator::adjointMatrix(A[i]*_qdot(i));
        }
        return;
    }

    void Liedynamics::ad_V_Link( VectorXd _qdot )
    {
        V.setZero();
        V.noalias() += LA_mat*_qdot;

        this->ad_V.setZero();

        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->ad_V.block<6, 6>(6 * i, 6 * i) = LieOperator::adjointMatrix(V.segment(6 * i, 6));
        }

        return;
    }

    void Liedynamics::Vdot_base( void )
    {
        VdotBase.setZero();
        this->VdotBase.segment(0, 6).noalias() += LieOperator::AdjointMatrix(LieOperator::inverse_SE3(pCoM->GetTMat(0, 1)))*grav;
        return;
    }

    void Liedynamics::PrepareDynamics( const VectorXd &_q, const VectorXd &_qdot )
    {
        if(isFirstRun == 0)
        {
            Inertia_Link();
            A_Link();
            Gamma_Link();
            isFirstRun = 1;
        }

        pCoM->PoEKinematics::HTransMatrix(_q);

        L_link();
        Vdot_base();

        LA_mat.setZero();
        LA_mat.noalias() += L_mat*A_mat;
        ad_Aqdot_Link(_qdot);
        ad_V_Link(_qdot);
    }

    void Liedynamics::C_Matrix( MatrixXd &_C )
    {
        _C.resize(this->m_DoF, this->m_DoF);
        _C.setZero();
        _C = -LA_mat.transpose()*(Iner_mat*L_mat*ad_Aqd*Gamma_mat - ad_V.transpose()*Iner_mat)*LA_mat;
        return;
    }

    void Liedynamics::C_Vector( VectorXd &_C, VectorXd &_qdot )
    {
        _C.resize(this->m_DoF);
        _C.setZero();
        _C.noalias() += (-LA_mat.transpose()*((Iner_mat*L_mat*ad_Aqd*Gamma_mat - ad_V.transpose()*Iner_mat)*(LA_mat*_qdot)));
        return;
    }

    void Liedynamics::G_Matrix( VectorXd &_G )
    {
        _G.resize(this->m_DoF);
        _G.setZero();
        _G.noalias() += (LA_mat.transpose()*(Iner_mat*(L_mat*VdotBase)));
        return;
    }

    void Liedynamics::MG_Mat_Joint( MatrixXd &_M, VectorXd&_G )
    {
        _M.resize(this->m_DoF,this->m_DoF);
        _M.setZero();
        _G.resize(this->m_DoF);
        _G.setZero();

        _M.noalias() += LA_mat.transpose()*Iner_mat*L_mat*A_mat;
        _G.noalias() += (LA_mat.transpose()*(Iner_mat*(L_mat*VdotBase)));

        return;
    }

    void Liedynamics::MG_Mat_Task(const MatrixXd &_M, const VectorXd &_G, MatrixXd &_Mx, VectorXd &_Gx)
    {
        pLink->GetAnalyticJacobian(this->Jacobian_mat);
        pLink->GetpinvJacobian(this->pinvJacobian_mat);
        //pLink->->GetScaledTransJacobian(this->pinvJacobian_mat);

        _Mx.resize(6*this->m_NumChain, 6*this->m_NumChain);
        _Mx.setZero();
        _Mx.noalias() += pinvJacobian_mat.transpose()*_M*pinvJacobian_mat;

        _Gx.resize(6*this->m_NumChain);
        _Gx.setZero();
        _Gx.noalias() += pinvJacobian_mat.transpose()*_G;
        return;
    }

    void Liedynamics::Mdot_Matrix( MatrixXd &_Mdot )
    {
        _Mdot.resize(m_DoF, m_DoF);
        _Mdot.setZero();
        _Mdot.noalias() += -LA_mat.transpose()*Gamma_mat.transpose()*ad_Aqd.transpose()*L_mat.transpose()*Iner_mat*LA_mat;
        _Mdot.noalias() += -LA_mat.transpose()*Iner_mat*L_mat*ad_Aqd*Gamma_mat*LA_mat;
        return;
    }

}
