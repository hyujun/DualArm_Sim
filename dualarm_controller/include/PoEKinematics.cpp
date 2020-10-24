#include "PoEKinematics.h"

namespace HYUMotionKinematics {

PoEKinematics::PoEKinematics():m_NumChain(1),m_DoF(6),isInfoUpdated(0)
{
	this->mBodyJacobian.resize(6*this->m_NumChain, this->m_DoF);
	this->mSpaceJacobian.resize(6*this->m_NumChain, this->m_DoF);
	this->mAnalyticJacobian.resize(6*this->m_NumChain, this->m_DoF);
	Theta=0;
}

PoEKinematics::PoEKinematics( const MatrixXi &_ChainMat ):isInfoUpdated(0)
{
	ChainMatrix = _ChainMat;

	this->m_DoF = ChainMatrix.cols();
	this->m_NumChain = ChainMatrix.rows();

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
		this->Arr[i].resize(ChainJointCount[i]);
	}

	this->mBodyJacobian.resize(6*this->m_NumChain, this->m_DoF);
	this->mSpaceJacobian.resize(6*this->m_NumChain, this->m_DoF);
	this->mAnalyticJacobian.resize(6*this->m_NumChain, this->m_DoF);

	Theta=0;
}

PoEKinematics::~PoEKinematics()
{

}

void PoEKinematics::UpdateKinematicInfo( Vector3d _w, Vector3d _p, Vector3d _l, int _link_num )
{
	M[_link_num] = GetM(_l);

	v_se3[_link_num] = GetTwist(_w, GetV(_w, _p));

	isInfoUpdated = 1;
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

se3 PoEKinematics::GetTwist( const Vector3d &_w, const Vector3d &_v )
{
	se3_Tmp.segment(0,3) = _w;
	se3_Tmp.segment(3,3) = _v;

	return se3_Tmp;
}

void PoEKinematics::HTransMatrix( const double *_q )
{
	int TCounter;

	for (int end = 0; end < this->m_DoF; ++end)
	{
		Exp_S[end] = SE3Matrix(v_se3[end], _q[end]);
	}

	for(int i=0; i < this->m_NumChain; ++i)
	{
		SE3_Tmp.setIdentity();
		TCounter=0;
		Arr[i].setZero();

		for(int j=0; j < this->m_DoF; j++)
		{
			if(ChainMatrix(i,j) == 1)
			{
				SE3_Tmp *= Exp_S[j];

				Arr[i](TCounter) = j+1;
				TCounter++;

				T[0][j+1].setZero();
				T[0][j+1].noalias() += SE3_Tmp*M[j];
			}
		}

		for(int k=0; k<TCounter; k++)
		{
			for(int l=(k+1); l<TCounter; l++)
			{
				T[Arr[i](k)][Arr[i](l)].setZero();
				T[Arr[i](k)][Arr[i](l)].noalias() += inverse_SE3(T[0][Arr[i](k)])*T[0][Arr[i](l)];
			}
		}
	}

	return;
}

void PoEKinematics::PrepareJacobian( const double *_q )
{
	HTransMatrix(_q);
	SpaceJacobian();
	SpaceToBodyJacobian();
	AnalyticJacobian();
	ScaledTransJacobian();
	return;
}

void PoEKinematics::SpaceJacobian( void )
{
	mSpaceJacobian.setZero();
	for(int i=0; i < this->m_NumChain; ++i)
	{
		SE3_Tmp.setIdentity();

		for(int j=0; j < this->m_DoF; ++j)
		{
			if(ChainMatrix(i,j) == 1)
			{
				SE3_Tmp*=Exp_S[j];
				if(j == 0)
				{
					mSpaceJacobian.block(6*i, j, 6, 1) = v_se3[j];
				}
				else
				{
					mSpaceJacobian.block(6*i, j, 6, 1).noalias() += AdjointMatrix(SE3_Tmp)*v_se3[j];
				}
			}
		}
	}

	return;
}

void PoEKinematics::SpaceToBodyJacobian( void )
{
	mBodyJacobian.setZero();
	for(int i=0; i < this->m_NumChain; i++)
	{
		mBodyJacobian.block(6*i,0,6,this->m_DoF).noalias() += AdjointMatrix(inverse_SE3(T[0][JointEndNum[i]]))*mSpaceJacobian.block(6*i,0,6,this->m_DoF);
	}
	return;
}

void PoEKinematics::GetSpaceJacobianDot(MatrixXd &_sJacobianDot)
{
  _sJacobianDot.setZero(6, this->m_DoF);
  for(int i = 0; i < this->m_DoF; i++)
  {
    for(int j=0; j < this->m_DoF; j++)
    {
      if(i > j)
      {
        _sJacobianDot.block(i, 6*j, 6, 1) = adjointMatrix(mSpaceJacobian.col(i))*mSpaceJacobian.col(j);
      }
    }
  }
}

void PoEKinematics::GetBodyJacobianDot(MatrixXd &_bJacobianDot )
{
  _bJacobianDot.setZero(6, this->m_DoF);
  for(int i = 0; i < this->m_DoF; i++)
  {
    for(int j=0; j < this->m_DoF; j++)
    {
      if(i < j)
      {
        _bJacobianDot.block(i, 6*j, 6, 1) = adjointMatrix(mBodyJacobian.col(i))*mBodyJacobian.col(j);
      }
    }
  }
}

void PoEKinematics::AnalyticJacobian( void )
{
	Mat_Tmp.resize(6*this->m_NumChain, 6*this->m_NumChain);
	Mat_Tmp.setZero();

    mAnalyticJacobian.setZero();

	for(int i=0; i < this->m_NumChain; i++)
	{
		LogSO3(T[0][JointEndNum[i]].block(0,0,3,3), Omega, Theta);
		if(abs(Theta) <= 1e-7)
		{
			Mat_Tmp.block(6*i,6*i,3,3) = Matrix3d::Identity();
		}
		else
		{
            Mat_Tmp.block(6*i,6*i,3,3) = Matrix3d::Identity();
            //Mat_Tmp.block(6*i,6*i,3,3) = LieOperator::SkewMatrix(Omega);
			//r = Omega*fmod(Theta,M_PI);
			//r = Omega;
			//Mat_Tmp.block(6*i,6*i,3,3) = Matrix3d::Identity() -  1/2*LieOperator::SkewMatrix(r) - ((1/r.squaredNorm() + (1+cos(r.norm())/(2*r.norm()*sin(r.norm()))))*LieOperator::SkewMatrixSquare(r));
		}
		Mat_Tmp.block((6*i+3),(6*i+3),3,3) = T[0][JointEndNum[i]].block(0,0,3,3);

		mAnalyticJacobian.block(6*i, 0, 3, this->m_DoF).noalias() += mSpaceJacobian.block(6*i, 0, 3, this->m_DoF);
        mAnalyticJacobian.block(6*i+3, 0, 3, this->m_DoF) = T[0][JointEndNum[i]].block(0,0,3,3)*mBodyJacobian.block(6*i+3, 0, 3, this->m_DoF);
    }


	//mAnalyticJacobian.noalias() += Mat_Tmp*mBodyJacobian;
	return;
}

void PoEKinematics::GetpInvJacobianWOOrientation(Eigen::MatrixXd & _pInvJacobian)
{
    MatrixXd posJacobian(3*m_NumChain, m_DoF);
    for(int i=0; i<m_NumChain; i++)
    {
        //posJacobian(0, 0, )
    }
}

void PoEKinematics::GetpinvJacobian( MatrixXd &_pinvJacobian )
{
	_pinvJacobian = mAnalyticJacobian.completeOrthogonalDecomposition().pseudoInverse();

	/*
	Eigen::BDCSVD<MatrixXd> svd(mAnalyticJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.setThreshold(1e-5);
	_pinvJacobian = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
	*/
	return;
}

void PoEKinematics::ScaledTransJacobian(void)
{
	ScaledFactor.resize(this->mAnalyticJacobian.cols());
	Vec_Tmp.resize(this->mAnalyticJacobian.rows());

	for(int i=0; i<this->mAnalyticJacobian.cols(); i++)
	{
		Vec_Tmp = this->mAnalyticJacobian.col(i);
		ScaledFactor(i) = Vec_Tmp.squaredNorm()-1;
	}

	mScaledTransJacobian = ScaledFactor.asDiagonal()*this->mAnalyticJacobian.transpose();
	return;
}

void PoEKinematics::GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian )
{
	_ScaledTransJacobian = mScaledTransJacobian;
	return;
}

void PoEKinematics::GetManipulability( double *_TaskEigen, double *_OrientEigen )
{
	Eigen::JacobiSVD<MatrixXd> SVDsolver;
	for(int i=0; i<this->m_NumChain; i++)
	{
		Mat_Tmp = this->mAnalyticJacobian.block(6*i,0, 3,this->m_DoF);
		SVDsolver.compute(Mat_Tmp*Mat_Tmp.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
		_OrientEigen[i] = SVDsolver.singularValues().maxCoeff() / SVDsolver.singularValues().minCoeff();

		Mat_Tmp = this->mAnalyticJacobian.block((6*i+3),0, 3,this->m_DoF);
		SVDsolver.compute(Mat_Tmp*Mat_Tmp.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
		_TaskEigen[i] = SVDsolver.singularValues().maxCoeff() / SVDsolver.singularValues().minCoeff();
	}
	return;
}

double PoEKinematics::GetManipulabilityMeasure()
{
    return sqrt((mAnalyticJacobian*mAnalyticJacobian.transpose()).determinant());
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
	//return T[0][JointEndNum[_EndPosition]];
    return T[0][_EndPosition];
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
	return;
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
	return;
}

}
