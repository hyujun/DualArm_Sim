#include "PoEKinematics.h"

namespace HYUMotionKinematics {

    PoEKinematics::PoEKinematics():m_NumChain(1),m_DoF(6),WpInv_epsilon_left(0.001), WpInv_epsilon_right(0.001)
    {
        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);
    }

    PoEKinematics::PoEKinematics( const MatrixXi &_ChainMat ):WpInv_epsilon_left(0.001), WpInv_epsilon_right(0.001)
    {
        ChainMatrix = _ChainMat;

        this->m_DoF = ChainMatrix.cols();
        this->m_NumChain = ChainMatrix.rows();

        int dofCounter;

        for(int i=0; i<this->m_NumChain; ++i)
        {

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
            dofCounter = 0;
            for(int k=0; k < this->m_DoF; k++)
            {
                if(ChainMatrix(i,k) == 1)
                {
                    this->Arr[i](dofCounter) = k+1;
                    dofCounter++;
                }
            }
        }

        qLimit_High.setZero(m_DoF);
        qLimit_Low.setZero(m_DoF);

        for(int l=0; l<m_DoF; l++)
        {
            qLimit_Low(l) = joint_limit.Low[l];
            qLimit_High(l) = joint_limit.High[l];
        }

        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);
    }

    PoEKinematics::~PoEKinematics()
    {

    }

    void PoEKinematics::UpdateKinematicInfo( const Vector3d &_w, const Vector3d &_p, const Vector3d &_Rot, const Vector3d &_l, const int _link_num )
    {
        M[_link_num] = GetM(_Rot, _l);

        v_se3[_link_num] = GetTwist(_w, GetV(_w, _p));
    }

    Vector3d PoEKinematics::GetV( const Vector3d &_w, const Vector3d &_p )
    {
        return -SkewMatrix(_w)*_p;
    }

    SE3 PoEKinematics::GetM( const Vector3d &_Rot, const Vector3d &_link )
    {
        SE3_Tmp.setIdentity();
        Eigen::Quaternion<double> q;
        q = Eigen::AngleAxisd(_Rot(2), Vector3d::UnitZ())*Eigen::AngleAxisd(_Rot(1), Vector3d::UnitY())*Eigen::AngleAxisd(_Rot(0), Vector3d::UnitX());
        SE3_Tmp.block(0,0,3,3) = q.matrix();
        SE3_Tmp.block(0,3,3,1) = _link;
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
        mSpaceJacobian.setZero(6*m_NumChain, m_DoF);
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
        mBodyJacobian.setZero(6*m_NumChain, m_DoF);
        for(int i=0; i < this->m_NumChain; i++)
        {
            mBodyJacobian.block(6*i,0,6,this->m_DoF).noalias() +=
                    AdjointMatrix(inverse_SE3(T[0][JointEndNum[i]]))*mSpaceJacobian.block(6*i,0,6,this->m_DoF);
        }
    }

    void PoEKinematics::BodyJacobianDot( const VectorXd &_qdot )
    {
        mBodyJacobianDot.setZero(6*m_NumChain, this->m_DoF);
        adjoint adTmp;
        int RowCount[2]={0,0};

        for(int j = 0; j < this->m_NumChain; j++)
        {
            for(int i=0; i<m_DoF; i++)
            {
                if(ChainMatrix(j,i) == 1)
                {
                    RowCount[j]++;
                    adTmp = adjointMatrix(mBodyJacobian.col(i).segment(6*j,6));
                    for(int k=RowCount[j]; k<ChainJointCount[j]; k++)
                    {
                        mBodyJacobianDot.col(i).segment(6*j,6).noalias() +=
                                adTmp*(mBodyJacobian.col(Arr[j](k)-1).segment(6*j,6)*_qdot(Arr[j](k)-1));
                    }
                }
            }
        }
    }

    void PoEKinematics::GetBodyJacobianDot(MatrixXd &_BodyJacobianDot)
    {
        _BodyJacobianDot = mBodyJacobianDot;
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

    void PoEKinematics::AnalyticJacobianDot( const VectorXd &_qdot )
    {
        mAnalyticJacobianDot.setZero(6*m_NumChain, m_DoF);
        BodyJacobianDot(_qdot);
        Matrix3d rot_tmp;
        Vector3d vec_tmp;

        for(int i=0; i<m_NumChain; i++)
        {
            rot_tmp.setZero();
            vec_tmp.setZero();
            for(int j=0; j<m_DoF; j++)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    vec_tmp.noalias() += mSpaceJacobian.col(j).segment(6*i,3)*_qdot(j);
                }
            }
            rot_tmp = SkewMatrix(vec_tmp);
            mAnalyticJacobianDot.block(6*i, 0, 3, this->m_DoF).noalias() +=
                    rot_tmp*mAnalyticJacobian.block(6*i, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i, 0, 3, this->m_DoF).noalias() +=
                    GetForwardKinematicsSO3(JointEndNum[i])*mBodyJacobianDot.block(6*i, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i+3, 0, 3, this->m_DoF).noalias() +=
                    rot_tmp*mAnalyticJacobian.block(6*i+3, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i+3, 0, 3, this->m_DoF).noalias() +=
                    GetForwardKinematicsSO3(JointEndNum[i])*mBodyJacobianDot.block(6*i+3, 0, 3, this->m_DoF);
        }
    }

    void PoEKinematics::GetAnalyticJacobianDot( const VectorXd &_qdot, MatrixXd &_AnalyticJacobianDot )
    {
        AnalyticJacobianDot(_qdot);
        _AnalyticJacobianDot = mAnalyticJacobianDot;
    }

    void PoEKinematics::pInvJacobian()
    {
        mpInvJacobin.setZero(16,12);
        //mpInvJacobin.block(0,0,9,6) = mAnalyticJacobian.block(0,0,6,9).completeOrthogonalDecomposition().pseudoInverse();
        //mpInvJacobin.block(0,6,16,6) = mAnalyticJacobian.block(6,0,6,16).completeOrthogonalDecomposition().pseudoInverse();
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
        for(int i=0; i<m_DoF; i++)
        {
            ScaledFactor(i) = 1.0/mAnalyticJacobian.col(i).squaredNorm();
        }

        mScaledTransJacobian.setZero(m_DoF,6*m_NumChain);
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

    void PoEKinematics::DampedpInvJacobian(MatrixXd &_TargetMatrix, const double sigma)
    {
        Mat_Tmp.setZero(12,12);
        Mat_Tmp.noalias() += _TargetMatrix*_TargetMatrix.transpose();
        Mat_Tmp.noalias() += sigma*Eigen::Matrix<double, 12, 12>::Identity();

        mDampedpInvJacobian.setZero( m_DoF,6*m_NumChain );
        mDampedpInvJacobian.noalias() += _TargetMatrix.transpose()*Mat_Tmp.inverse();
    }

    void PoEKinematics::GetDampedpInvJacobian(MatrixXd &_TargetMat, MatrixXd &_DampedpInvJacobian)
    {
        DampedpInvJacobian(_TargetMat, 0.0025);
        _DampedpInvJacobian = mDampedpInvJacobian;
    }

    void PoEKinematics::BlockpInvJacobian( Matrix<double, 6, Dynamic> &_Jacobian1, Matrix<double, 6, Dynamic> &_Jacobian2 )
    {
        MatrixXd P1 = Matrix<double, 16, 16>::Identity();
        P1.noalias() += -_Jacobian1.transpose()*(_Jacobian1*_Jacobian1.transpose()).inverse()*_Jacobian1;
        MatrixXd P2 = Matrix<double, 16, 16>::Identity();
        P2.noalias() += -_Jacobian2.transpose()*(_Jacobian2*_Jacobian2.transpose()).inverse()*_Jacobian2;

        mBlockpInvJacobian.setZero(m_DoF, 6*m_NumChain);
        mBlockpInvJacobian.block(0,0,16,6) = (_Jacobian1*P2).completeOrthogonalDecomposition().pseudoInverse();
        mBlockpInvJacobian.block(0,6,16,6) = (_Jacobian2*P1).completeOrthogonalDecomposition().pseudoInverse();
    }

    void PoEKinematics::GetBlockpInvJacobian(MatrixXd &_BlockpInvJacobian)
    {
        Matrix<double, 6, Dynamic> J1 = mAnalyticJacobian.block(0,0,6,16);
        Matrix<double, 6, Dynamic> J2 = mAnalyticJacobian.block(6,0,6,16);
        BlockpInvJacobian(J1, J2);
        _BlockpInvJacobian = mBlockpInvJacobian;
    }

    void PoEKinematics::WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat )
    {

        WpInv_epsilon_left = 0.001;
        WpInv_epsilon_right = 0.0081;

        mWeightDampedpInvJacobian.setZero(16,12);

        VectorXd r1_right, r2_right, r1_left, r2_left;
        r2_right = _rdot.segment(0,3);  //rotation error for right-arm
        r1_right = _rdot.segment(3,3);  //translation error for right-arm
        r2_left = _rdot.segment(6,3);   //rotation error for left-arm
        r1_left = _rdot.segment(9,3);   //translation error for left-arm

        // analytic jacobian devided into left-arm jacobian & right-arm jacobian
        MatrixXd J_left1, J_right1;
        J_right1 = mAnalyticJacobian.block(0,0,6,16);
        J_left1 = mAnalyticJacobian.block(6,0,6,16);

        MatrixXd P1 = Matrix<double, 16, 16>::Identity();
        P1.noalias() += -J_right1.transpose()*(J_right1*J_right1.transpose()).inverse()*J_right1;
        MatrixXd P2 = Matrix<double, 16, 16>::Identity();
        P2.noalias() += -J_left1.transpose()*(J_left1*J_left1.transpose()).inverse()*J_left1;

        MatrixXd J_left, J_right;
        J_right = J_right1*P2;
        J_left = J_left1*P1;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for right-arm
        MatrixXd J1, J2;
        J1 = J_right.block(3,0,3,16);
        J2 = J_right.block(0,0,3,16);

        MatrixXd W = Matrix<double, 16, 16>::Zero();
        W.noalias() += J1.transpose()*J1;
        W.noalias() += J2.transpose()*J2;
        W.noalias() += WpInv_epsilon_right*_WeightMat;
        MatrixXd W_inv = W.inverse();
        //MatrixXd W_inv = W.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd Y = Matrix<double, 3, 3>::Zero();
        Y.noalias() += J1*W_inv*J1.transpose();
        MatrixXd Y_inv = Y.inverse();
        //MatrixXd Y_inv = Y.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd Z11 = W_inv;
        Z11.noalias() += -W_inv*(J1.transpose()*((Y_inv*J1)*W_inv));
        MatrixXd Z12 = Matrix<double, 16, 3>::Zero();
        Z12.noalias() += W_inv*(J1.transpose()*Y_inv);
        MatrixXd Z21 = Matrix<double, 3, 16>::Zero();
        Z21.noalias() += (Y_inv*J1)*W_inv;
        MatrixXd Z22 = Matrix<double, 3, 3>::Identity();
        Z22.noalias() += -Y_inv;

        MatrixXd J_WpInv_right = Matrix<double,16,6>::Zero();
        J_WpInv_right.block(0,0,16,3).noalias() += Z11*J2.transpose();
        J_WpInv_right.block(0,3,16,3) = Z12;
        lambda_right = -Z21*J2.transpose()*r2_right -Z22*r1_right;
        //WpInv_epsilon_right = lambda_right.norm();
        //WpInv_epsilon_right = tanh(lambda_right.norm());
        mWeightDampedpInvJacobian.block(0,0,16,6) = J_WpInv_right;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for left-arm
        J1 = J_left.block(3,0,3,16);
        J2 = J_left.block(0,0,3,16);

        W.setZero(16,16);
        W.noalias() += J1.transpose()*J1;
        W.noalias() += J2.transpose()*J2;
        W.noalias() += WpInv_epsilon_left*_WeightMat;
        W_inv = W.inverse();
        //W_inv = W.completeOrthogonalDecomposition().pseudoInverse();

        Y.setZero(3,3);
        Y.noalias() += J1*W_inv*J1.transpose();
        Y_inv = Y.inverse();
        //Y_inv = Y.completeOrthogonalDecomposition().pseudoInverse();

        Z11 = W_inv;
        Z11.noalias() += -W_inv*(J1.transpose()*((Y_inv*J1)*W_inv));
        Z12.setZero(16,3);
        Z12.noalias() += W_inv*(J1.transpose()*Y_inv);
        Z21.setZero(3,16);
        Z21.noalias() += (Y_inv*J1)*W_inv;
        Z22.setIdentity(3,3);
        Z22.noalias() += -Y_inv;

        MatrixXd J_WpInv_left = Matrix<double,16,6>::Zero();
        J_WpInv_left.block(0,0,16,3).noalias() += Z11*J2.transpose();
        J_WpInv_left.block(0,3,16,3) = Z12;
        lambda_left = -Z21*J2.transpose()*r2_left -Z22*r1_left;
        //WpInv_epsilon_left = lambda_left.norm();
        //WpInv_epsilon_left = tanh(lambda_left.norm());
        mWeightDampedpInvJacobian.block(0,6,16,6) = J_WpInv_left;
    }

    void PoEKinematics::GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_WDampedpInvJacobian )
    {
        WeightpInvJacobian(_rdot, _WeightMat);
        _WDampedpInvJacobian = mWeightDampedpInvJacobian;
    }

    void PoEKinematics::WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, const MatrixXd &_TargetMat )
    {

        WpInv_epsilon_left = 1.0;
        WpInv_epsilon_right = 1.0;

        mWeightDampedpInvJacobian.setZero(16,12);

        VectorXd r1_right, r2_right, r1_left, r2_left;
        r2_right = _rdot.segment(0,3);  //rotation error for right-arm
        r1_right = _rdot.segment(3,3);  //translation error for right-arm
        r2_left = _rdot.segment(6,3);   //rotation error for left-arm
        r1_left = _rdot.segment(9,3);   //translation error for left-arm

        // analytic jacobian devided into left-arm jacobian & right-arm jacobian
        MatrixXd J_left1, J_right1;
        J_right1 = _TargetMat.block(0,0,6,16);
        J_left1 = _TargetMat.block(6,0,6,16);

        MatrixXd P1 = Matrix<double, 16, 16>::Identity();
        P1.noalias() += -J_right1.transpose()*(J_right1*J_right1.transpose()).inverse()*J_right1;
        MatrixXd P2 = Matrix<double, 16, 16>::Identity();
        P2.noalias() += -J_left1.transpose()*(J_left1*J_left1.transpose()).inverse()*J_left1;

        MatrixXd J_left, J_right;
        J_right = J_right1*P2;
        J_left = J_left1*P1;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for right-arm
        MatrixXd J1, J2;
        J1 = J_right.block(3,0,3,16);
        J2 = J_right.block(0,0,3,16);

        MatrixXd W = Matrix<double, 16, 16>::Zero();
        W.noalias() += J1.transpose()*J1;
        W.noalias() += J2.transpose()*J2;
        W.noalias() += WpInv_epsilon_right*_WeightMat;
        MatrixXd W_inv = W.inverse();
        //MatrixXd W_inv = W.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd Y = Matrix<double, 3, 3>::Zero();
        Y.noalias() += J1*W_inv*J1.transpose();
        MatrixXd Y_inv = Y.inverse();
        //MatrixXd Y_inv = Y.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd Z11 = W_inv;
        Z11.noalias() += -W_inv*(J1.transpose()*((Y_inv*J1)*W_inv));
        MatrixXd Z12 = Matrix<double, 16, 3>::Zero();
        Z12.noalias() += W_inv*(J1.transpose()*Y_inv);
        MatrixXd Z21 = Matrix<double, 3, 16>::Zero();
        Z21.noalias() += (Y_inv*J1)*W_inv;
        MatrixXd Z22 = Matrix<double, 3, 3>::Identity();
        Z22.noalias() += -Y_inv;

        MatrixXd J_WpInv_right = Matrix<double,16,6>::Zero();
        J_WpInv_right.block(0,0,16,3).noalias() += Z11*J2.transpose();
        J_WpInv_right.block(0,3,16,3) = Z12;
        lambda_right = -Z21*J2.transpose()*r2_right -Z22*r1_right;
        //WpInv_epsilon_right = lambda_right.norm();
        //WpInv_epsilon_right = tanh(lambda_right.norm());
        mWeightDampedpInvJacobian.block(0,0,16,6) = J_WpInv_right;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for left-arm
        J1 = J_left.block(3,0,3,16);
        J2 = J_left.block(0,0,3,16);

        W.setZero(16,16);
        W.noalias() += J1.transpose()*J1;
        W.noalias() += J2.transpose()*J2;
        W.noalias() += WpInv_epsilon_left*_WeightMat;
        W_inv = W.inverse();
        //W_inv = W.completeOrthogonalDecomposition().pseudoInverse();

        Y.setZero(3,3);
        Y.noalias() += J1*W_inv*J1.transpose();
        Y_inv = Y.inverse();
        //Y_inv = Y.completeOrthogonalDecomposition().pseudoInverse();

        Z11 = W_inv;
        Z11.noalias() += -W_inv*(J1.transpose()*((Y_inv*J1)*W_inv));
        Z12.setZero(16,3);
        Z12.noalias() += W_inv*(J1.transpose()*Y_inv);
        Z21.setZero(3,16);
        Z21.noalias() += (Y_inv*J1)*W_inv;
        Z22.setIdentity(3,3);
        Z22.noalias() += -Y_inv;

        MatrixXd J_WpInv_left = Matrix<double,16,6>::Zero();
        J_WpInv_left.block(0,0,16,3).noalias() += Z11*J2.transpose();
        J_WpInv_left.block(0,3,16,3) = Z12;
        lambda_left = -Z21*J2.transpose()*r2_left -Z22*r1_left;
        //WpInv_epsilon_left = lambda_left.norm();
        //WpInv_epsilon_left = tanh(lambda_left.norm());
        mWeightDampedpInvJacobian.block(0,6,16,6) = J_WpInv_left;
    }

    void PoEKinematics::GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_TargetMat, MatrixXd &_WDampedpInvJacobian )
    {
        WeightpInvJacobian(_rdot, _WeightMat, _TargetMat);
        _WDampedpInvJacobian = mWeightDampedpInvJacobian;
    }

    void PoEKinematics::GetWDampedpInvLambda(VectorXd *lambda)
    {
        lambda[0] = lambda_right;
        lambda[1] = lambda_left;
    }

    void PoEKinematics::RelativeJacobian( const int From, const int To )
    {
        Matrix<double, 6, 6> Body2Analyic = Eigen::Matrix<double, 6, 6>::Zero();
        Matrix<double, 6, 16> RelJacTmp = Eigen::Matrix<double, 6, 16>::Zero();
        mRelativeJacobian.setZero(6, m_DoF);
        if( From == 0 && To == 1 )
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
        RelativeJacobian(0,1); // right to left
        _RelativeJacobian = mRelativeJacobian;
    }

    void PoEKinematics::RelativeJacobianDot(const VectorXd &_qdot)
    {
        mRelativeBodyJacobianDot.setZero(6,m_DoF);
        mRelativeJacobianDot.setZero(6,m_DoF);
        adjoint adTmp;
        Adjoint AdTmp;
        AdTmp = AdjointMatrix(inverse_SE3(T[0][JointEndNum[1]]));
        for(int i=3; i<=8; i++)
        {
            adTmp = adjointMatrix(mSpaceJacobian.col(i).segment(0,6));
            for(int k=2; k<i; k++)
            {
                mRelativeBodyJacobianDot.col(i).noalias() +=
                        -AdTmp*(adTmp*(mSpaceJacobian.col(k).segment(0,6)*_qdot(k)));
            }
        }
        for(int j=9; j<=14; j++)
        {
            adTmp = adjointMatrix(mBodyJacobian.col(j).segment(6,6));
            for(int l=j+1; l<=15; l++)
            {
                mRelativeBodyJacobianDot.col(j).noalias() += adTmp*(mBodyJacobian.col(l).segment(6,6)*_qdot(l));
            }
        }

        Matrix<double, 6, 6> Body2Analyic = Eigen::Matrix<double, 6, 6>::Zero();
        Body2Analyic.block(3,3,3,3).noalias() += GetForwardKinematicsSO3(JointEndNum[0]).transpose()*GetForwardKinematicsSO3(JointEndNum[1]);
        Body2Analyic.block(0,0,3,3) = Body2Analyic.block(3,3,3,3);

        mRelativeJacobianDot.noalias() += Body2Analyic*mRelativeBodyJacobianDot;

        Matrix<double, 6, 6> adMat;
        for(int n=2; n<16;n++)
        {
            adMat.setZero();
            if(n>=2 && n<=8)
            {
                adMat.block(0,0,3,3).noalias() += SkewMatrix(mBodyJacobian.col(n).segment(0,3)*_qdot(n));
                adMat.block(3,3,3,3) = adMat.block(0,0,3,3);
            }
            else
            {
                adMat.block(0,0,3,3).noalias() += SkewMatrix(mSpaceJacobian.col(n).segment(6,3)*_qdot(n));
                adMat.block(3,3,3,3) = adMat.block(0,0,3,3);
            }
            mRelativeJacobianDot.col(n).noalias() += adMat*mRelativeJacobian.col(n);
        }
    }

    void PoEKinematics::GetRelativeJacobianDot(const VectorXd &_qdot, MatrixXd &_RelativeJacobianDot)
    {
        _RelativeJacobianDot.setZero(6, m_DoF);
        RelativeJacobianDot(_qdot);
        _RelativeJacobianDot = mRelativeJacobianDot;
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

    double PoEKinematics::GetManipulabilityMeasure()
    {
        return sqrt((mAnalyticJacobian*mAnalyticJacobian.transpose()).determinant());
    }

    void PoEKinematics::Getq0dotWithMM(const double &gain, VectorXd &q0dot)
    {
        q0dot.setZero(m_DoF);
        Matrix<double, 12, 16> dJbdq, dJadq;
        Matrix<double, 12, 12> MatTrace;

        auto ManipulabilityMeasure = GetManipulabilityMeasure();

        Matrix<double, 12,12> K, dKdq;
        K.setZero();
        K.block(0,0,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(3,3,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(6,6,3,3) = GetForwardKinematicsSO3(JointEndNum[1]);
        K.block(9,9,3,3) = GetForwardKinematicsSO3(JointEndNum[1]);

        MatrixXd pInvJacobian;
        GetpinvJacobian(pInvJacobian);

        int RowCount[2]={0,0};

        for(int i=1; i<m_DoF; i++)
        {
            dJbdq.setZero();
            dKdq.setZero();
            for(int j=0; j<m_NumChain; j++)
            {
                if(ChainMatrix(j,i) == 1)
                {
                    RowCount[j]++;
                    for(int k=0; k<RowCount[j]; k++)
                    {
                        dJbdq.col(Arr[j](k)-1).segment(6*j,6).noalias() +=
                                adjointMatrix(mBodyJacobian.col(Arr[j](k)-1).segment(6*j,6))*mBodyJacobian.col(i).segment(6*j,6);
                    }
                    dKdq.block(6*j,6*j,3,3).noalias() = SkewMatrix(mSpaceJacobian.col(i).segment(6*j,3));
                    dKdq.block(6*j+3,6*j+3,3,3) = dKdq.block(6*j,6*j,3,3) ;
                }
            }

            dJadq.setZero();
            dJadq.noalias() += dKdq*mAnalyticJacobian;
            dJadq.noalias() += K*dJbdq;

            MatTrace.setZero();
            MatTrace.noalias() += dJadq*pInvJacobian;

            q0dot(i) = gain*ManipulabilityMeasure*MatTrace.trace();
        }
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
        AngleAxis<double> rot;
        for(int i=0; i<this->m_NumChain; i++)
        {
            rot = GetForwardKinematicsSO3(JointEndNum[i]);
            _Axis[i] = rot.axis();
            _Angle[i] = rot.angle();
            //LogSO3(T[0][JointEndNum[i]].block(0,0,3,3), _Axis[i], _Angle[i]);
        }
    }

    void PoEKinematics::SO3toAngleAxis(const Matrix3d &_RotMat, Vector3d &_orientation)
    {
        AngleAxis<double> rot;
        rot = _RotMat;
        _orientation = rot.axis()*rot.angle();
    }

    void PoEKinematics::RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat)
    {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(_Yaw_rad, Vector3d::UnitZ())*Eigen::AngleAxisd(_Pitch_rad, Vector3d::UnitY())*Eigen::AngleAxisd(_Roll_rad, Vector3d::UnitX());
        _RotMat = q.toRotationMatrix();
    }

    void PoEKinematics::SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation )
    {
        q = _RotMat;

        _Orientation(0) = atan2(2.0*(q.x()*q.w()+q.y()*q.z()) , 1.0-2.0*(pow(q.x(),2) + pow(q.y(),2)) );

        double sinp = 2.0*( q.w()*q.y() - q.z()*q.x() );
        if(abs(sinp) >= 1)
            _Orientation(1) = copysign(M_PI_2, sinp);
        else
            _Orientation(1) = asin(sinp);

        _Orientation(2) = atan2( 2.0*(q.w()*q.z() + q.x()*q.y()) , 1.0-2.0*( pow(q.y(),2) + pow(q.z(),2) ) );
    }


}
