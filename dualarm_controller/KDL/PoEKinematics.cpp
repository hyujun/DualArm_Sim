#include "PoEKinematics.h"

namespace HYUMotionKinematics {

    PoEKinematics::PoEKinematics():m_NumChain(1),m_DoF(6)
    {
        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        Theta=0;
    }

    PoEKinematics::PoEKinematics( const MatrixXi &_ChainMat )
    {
        ChainMatrix = _ChainMat;

        this->m_DoF = ChainMatrix.cols();
        this->m_NumChain = ChainMatrix.rows();

        int dofCounter;

        for(int i=0; i<this->m_NumChain; ++i)
        {
            dofCounter = 0;
            ChainJointCount[i]=0;
            for(int j=(this->m_DoF-1); j>=0; --j)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    if(ChainJointCount[i] == 0)
                    {
                        JointEndNum[i] = j+1;
                    }
                    ChainJointCount[i]++;
                }
            }
            this->Arr[i].setZero(ChainJointCount[i]);

            for(int k=0; k < this->m_DoF; k++)
            {
                if(ChainMatrix(i,k) == 1)
                {
                    this->Arr[i](dofCounter) = k+1;
                    dofCounter++;
                }
            }
        }

        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);

        Theta=0;
    }

    PoEKinematics::~PoEKinematics()
    {

    }

    void PoEKinematics::UpdateKinematicInfo( const Vector3d &_w, const Vector3d &_p, const Vector3d &_l, const int _link_num )
    {
        M[_link_num] = GetM(_l);

        v_se3[_link_num] = GetTwist(_w, GetV(_w, _p));
    }

    Vector3d PoEKinematics::GetV( const Vector3d &_w, const Vector3d &_p )
    {
        return -SkewMatrix(_w)*_p;
    }

    SE3 PoEKinematics::GetM( const Vector3d &_link )
    {
        SE3_Tmp.setIdentity();
        SE3_Tmp.block<3, 1>(0, 3) = _link;
        return SE3_Tmp;
    }

    void PoEKinematics::SetTwist( const se3 &_Twist, const int _link_num )
    {
        v_se3[_link_num] = _Twist;
    }

    se3 PoEKinematics::GetTwist( const Vector3d &_w, const Vector3d &_v )
    {
        se3_Tmp.segment(0,3) = _w;
        se3_Tmp.segment(3,3) = _v;

        return se3_Tmp;
    }

    void PoEKinematics::HTransMatrix( const VectorXd &_q )
    {
        for (int end=0; end < m_DoF; end++)
        {
            Exp_S[end] = SE3Matrix(v_se3[end], _q(end));
        }

        for(int i=0; i < this->m_NumChain; ++i)
        {
            SE3_Tmp.setIdentity();

            for(int j=0; j < this->m_DoF; j++)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    SE3_Tmp *= Exp_S[j];

                    T[0][j+1].setZero();
                    T[0][j+1].noalias() += SE3_Tmp*M[j];
                }
            }
        }
    }

    void PoEKinematics::PrepareJacobian( const VectorXd &_q )
    {
        HTransMatrix(_q);
        SpaceJacobian();
        SpaceToBodyJacobian();
        AnalyticJacobian();
        pInvJacobian();
    }

    void PoEKinematics::SpaceJacobian()
    {
        mSpaceJacobian.setZero();
        for(int i=0; i < this->m_NumChain; ++i)
        {
            SE3_Tmp.setIdentity();

            for(int j=0; j < this->m_DoF; ++j)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    if(j == 0)
                    {
                        mSpaceJacobian.block(6*i, j, 6, 1) = v_se3[j];
                    }
                    else
                    {
                        mSpaceJacobian.block(6*i, j, 6, 1).noalias() += AdjointMatrix(SE3_Tmp)*v_se3[j];
                    }
                    SE3_Tmp*=Exp_S[j];
                }
            }
        }
    }

    void PoEKinematics::SpaceToBodyJacobian()
    {
        mBodyJacobian.setZero();
        for(int i=0; i < this->m_NumChain; i++)
        {
            mBodyJacobian.block(6*i,0,6,this->m_DoF).noalias() += AdjointMatrix(inverse_SE3(T[0][JointEndNum[i]]))*mSpaceJacobian.block(6*i,0,6,this->m_DoF);
        }
    }

    void PoEKinematics::GetSpaceJacobianDot( MatrixXd &_sJacobianDot )
    {
        _sJacobianDot.setZero(6, this->m_DoF);
        for(int i = 0; i < this->m_DoF; i++)
        {
            for(int j=0; j < this->m_DoF; j++)
            {
                if(i > j)
                {
                    _sJacobianDot.block(i, 6*j, 6, 1).noalias() += adjointMatrix(mSpaceJacobian.col(i))*mSpaceJacobian.col(j);
                }
            }
        }
    }

    void PoEKinematics::GetBodyJacobianDot( MatrixXd &_bJacobianDot )
    {
        _bJacobianDot.setZero(6, this->m_DoF);
        for(int i = 0; i < this->m_DoF; i++)
        {
            for(int j=0; j < this->m_DoF; j++)
            {
                if(i < j)
                {
                    _bJacobianDot.block(i, 6*j, 6, 1).noalias() += adjointMatrix(mBodyJacobian.col(i))*mBodyJacobian.col(j);
                }
            }
        }
    }

    void PoEKinematics::AnalyticJacobian()
    {
        mAnalyticJacobian.setZero(6*m_NumChain, m_DoF);
        int aJac_case=1;
        for(int i=0; i < this->m_NumChain; i++)
        {
            if(aJac_case == 0)
            {
                Mat_Tmp.setZero(6, 6);
                LogSO3(GetForwardKinematicsSO3(JointEndNum[i]), Omega, Theta);
                if(abs(Theta) <= 1e-2)
                {
                    Mat_Tmp.block(0,0,3,3) = Matrix3d::Identity();
                }
                else
                {
                    r = Omega*Theta;
                    Mat_Tmp.block(0,0,3,3).noalias() += Matrix3d::Identity();
                    Mat_Tmp.block(0,0,3,3).noalias() += 0.5*LieOperator::SkewMatrix(r);
                    double tmp = 1.0/r.squaredNorm() - (1.0+cos(r.norm()))/(2.0*r.norm()*sin(r.norm()));
                    Mat_Tmp.block(0,0,3,3).noalias() += tmp*LieOperator::SkewMatrixSquare(r);
                }
                Mat_Tmp.block(0,0,3,3) = GetForwardKinematicsSO3(JointEndNum[i]);
                Mat_Tmp.block(3,3,3,3) = GetForwardKinematicsSO3(JointEndNum[i]);
                mAnalyticJacobian.block(6*i,0,6,m_DoF).noalias() += Mat_Tmp*mBodyJacobian.block(6*i,0,6,m_DoF);
            }
            else
            {
                mAnalyticJacobian.block(6*i, 0, 3, this->m_DoF) = mSpaceJacobian.block(6*i, 0, 3, this->m_DoF);
                mAnalyticJacobian.block(6*i+3, 0, 3, this->m_DoF).noalias() += T[0][JointEndNum[i]].block(0,0,3,3)*mBodyJacobian.block(6*i+3, 0, 3, this->m_DoF);
            }
        }
    }

    void PoEKinematics::pInvJacobian()
    {
        mpInvJacobin = mAnalyticJacobian.completeOrthogonalDecomposition().pseudoInverse();
        /*
        Eigen::BDCSVD<MatrixXd> svd(mAnalyticJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        svd.setThreshold(1e-5);
        mpInvJacobin = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
        */
    }

    void PoEKinematics::GetpinvJacobian( MatrixXd &_pinvJacobian )
    {
        _pinvJacobian = mpInvJacobin;
    }

    void PoEKinematics::ScaledTransJacobian()
    {
        ScaledFactor.setZero(m_DoF);
        mScaledTransJacobian.setZero();

        for(int i=0; i<m_DoF; i++)
        {
            ScaledFactor(i) = 1.0/mAnalyticJacobian.squaredNorm();
        }

        mScaledTransJacobian.noalias() += ScaledFactor.asDiagonal()*mAnalyticJacobian.transpose();
    }

    void PoEKinematics::GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian )
    {
        ScaledTransJacobian();
        _ScaledTransJacobian = mScaledTransJacobian;
    }

    void PoEKinematics::DampedpInvJacobian(const double sigma)
    {
        Mat_Tmp.setZero(12,12);
        Mat_Tmp.noalias() += mAnalyticJacobian*mAnalyticJacobian.transpose();
        Mat_Tmp.noalias() += sigma*Eigen::Matrix<double, 12, 12>::Identity();

        mDampedpInvJacobian.setZero( m_DoF,6*m_NumChain );
        mDampedpInvJacobian.noalias() += mAnalyticJacobian.transpose()*Mat_Tmp.inverse();
    }

    void PoEKinematics::GetDampedpInvJacobian( MatrixXd &_DampedpInvJacobian )
    {
        DampedpInvJacobian(0.0025);
        _DampedpInvJacobian = mDampedpInvJacobian;
    }

    void PoEKinematics::GetBlockpInvJacobian( MatrixXd &_BlockpInvJacobian )
    {

    }

    void PoEKinematics::RelativeJacobian( const int From, const int To )
    {
        Matrix<double, 6, 6> Body2Analyic = Eigen::Matrix<double, 6, 6>::Identity();
        Matrix<double, 6, 16> RelJacTmp = Eigen::Matrix<double, 6, 16>::Zero();
        mRelativeJacobian.setZero(6, m_DoF);
        if(From == 0 && To == 1)
        {
            RelJacTmp.block(0, 2, 6, 7).noalias() += -AdjointMatrix(inverse_SE3(T[0][JointEndNum[To]]))*mSpaceJacobian.block(0, 2, 6, 7);
            RelJacTmp.block(0, 9, 6, 7) = mBodyJacobian.block(6, 9, 6, 7);
            Body2Analyic.block(3,3,3,3).noalias() += GetForwardKinematicsSO3(JointEndNum[From]).transpose()*GetForwardKinematicsSO3(JointEndNum[To]);
            Body2Analyic.block(0,0,3,3) = Body2Analyic.block(3,3,3,3);
        }
        mRelativeJacobian.noalias() += Body2Analyic*RelJacTmp;
    }

    void PoEKinematics::GetRelativeJacobian( MatrixXd &_RelativeJacobian )
    {
        RelativeJacobian(0,1);
        _RelativeJacobian = mRelativeJacobian;
    }

    void PoEKinematics::GetInverseConditionNumber( double *_InverseCondNumber )
    {
        for(int i=0; i<this->m_NumChain; i++)
        {
            Mat_Tmp = this->mAnalyticJacobian.block(6*i,0, 6,this->m_DoF);
            auto jac_svd = Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>>{Mat_Tmp};
            _InverseCondNumber[i] = jac_svd.singularValues().minCoeff() / jac_svd.singularValues().maxCoeff();
        }
    }

    double PoEKinematics::GetDAManipulabilityMeasure()
    {
        return sqrt((mAnalyticJacobian*mAnalyticJacobian.transpose()).determinant());
    }

    void PoEKinematics::Getq0dotWithMM(const double &gain, VectorXd &q0dot)
    {
        q0dot.setZero(m_DoF);
        MatrixXd dJbdq, MatTrace;
        adjoint adTmp;
        double ManipulabilityMeasure = GetDAManipulabilityMeasure();

        MatrixXd pInvJacobian;
        GetpinvJacobian(pInvJacobian);

        int RowCount[2]={0,0};

        for(int i=1; i<m_DoF; i++)
        {
            dJbdq.setZero(6*m_NumChain, m_DoF);
            MatTrace.setZero(6*m_NumChain,6*m_NumChain);

            for(int j=0; j<m_NumChain; j++)
            {
                if(ChainMatrix(j,i) == 1)
                {
                    RowCount[j]++;
                    adTmp.setZero();
                    adTmp.noalias() += -adjointMatrix(AdjointMatrix(inverse_SE3(M[JointEndNum[j]-1]))*v_se3[Arr[j](RowCount[j])-1]);
                    for(int k=0; k<RowCount[j]; k++)
                    {
                        dJbdq.col(Arr[j](ChainJointCount[j]-(k+1))-1).segment(6*j,6).noalias()
                                += adTmp*mBodyJacobian.col(Arr[j](ChainJointCount[j]-(k+1))-1).segment(6*j,6);
                    }
                }
            }

            MatTrace.noalias() += dJbdq*pInvJacobian;
            q0dot(i) = gain*ManipulabilityMeasure*MatTrace.trace();
        }
    }

    void PoEKinematics::GetTaskVelocity( double *_qdot, VectorXd *_TaskVelocity, int &_size )
    {
        _size = this->m_NumChain;

        Vec_Tmp.resize(6*this->m_NumChain);
        Vec_Tmp.setZero();
        Vec_Tmp.noalias() += mAnalyticJacobian*Map<VectorXd>(_qdot, this->m_DoF);

        for(int i=0; i<this->m_NumChain; i++)
        {
            _TaskVelocity[i] = Vec_Tmp.segment(6*i, 6);
        }
        return;
    }

    void PoEKinematics::GetForwardKinematics( Vector3d *_Position, Vector3d *_Orientation, int &_NumChain )
    {
        _NumChain = this->m_NumChain;

        for(int i=0; i<this->m_NumChain; i++)
        {
            _Position[i].setZero();
            _Position[i] = T[0][JointEndNum[i]].block(0,3,3,1);
            SO3toRollPitchYaw(T[0][JointEndNum[i]].block(0,0,3,3), _Orientation[i]);
        }

        return;
    }

    SE3 PoEKinematics::GetForwardKinematicsSE3( const int &_EndPosition ) const
    {
        return T[0][_EndPosition];
    }

    SO3 PoEKinematics::GetForwardKinematicsSO3(const int &_EndPosition) const
    {
        return T[0][_EndPosition].block(0, 0, 3, 3);
    }

    void PoEKinematics::GetAngleAxis( Vector3d *_Axis, double *_Angle, int &_NumChain )
    {
        _NumChain = this->m_NumChain;

        for(int i=0; i<this->m_NumChain; i++)
        {
            LogSO3(T[0][JointEndNum[i]].block(0,0,3,3), _Axis[i], _Angle[i]);
        }

        return;
    }

    void PoEKinematics::RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat)
    {
        _RotMat = Eigen::AngleAxisd(_Yaw_rad, Vector3d::UnitZ())*Eigen::AngleAxisd(_Pitch_rad, Vector3d::UnitY())*Eigen::AngleAxisd(_Roll_rad, Vector3d::UnitX());
    }

    void PoEKinematics::SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation )
    {
        q = _RotMat;

        _Orientation(0) = atan2(2.0*(q.x()*q.w()+q.y()*q.z()) , 1.0-2.0*(pow(q.x(),2) + pow(q.y(),2)) );

        double sinp = 2.0*( q.w()*q.y() - q.z()*q.x() );
        if(abs(sinp) >= 1)
            _Orientation(1) = copysign(M_PI/2, sinp);
        else
            _Orientation(1) = asin(sinp);

        _Orientation(2) = atan2( 2.0*(q.w()*q.z() + q.x()*q.y()) , 1.0-2.0*( pow(q.y(),2) + pow(q.z(),2) ) );
    }

}
