/*
 * Trajectory.cpp
 *
 *  Created on: Nov 20, 2018
 *      Author: spec
 */

#include "Trajectory.h"

namespace HYUControl {

Trajectory::Trajectory()
:m_isReady(0)
{

}


Trajectory::~Trajectory() {

}

void Trajectory::SetPoly5th( const double &_CurrentTime, Eigen::VectorXd &_StartPos, Eigen::VectorXd &_StartVel, Eigen::VectorXd &_FinalPos, double &_Duration, int &_NumJoint )
{
	TrajDuration = _Duration;
	TrajInitTime = _CurrentTime;

	NumberJoint = _NumJoint;

	this->Coefficient.resize(6, NumberJoint);
	Coefficient.setZero();
	this->StateVec.resize(6, NumberJoint);
	StateVec.setZero();

	m_cof << 1.0, 0.0, 					0.0, 						0.0, 						0.0, 						0.0,
			 0.0, 1.0, 					0.0, 						0.0, 						0.0, 						0.0,
			 0.0, 0.0, 					2.0, 						0.0, 						0.0, 						0.0,
			 1.0, pow(TrajDuration,1), 	pow(TrajDuration,2), 		pow(TrajDuration,3), 		pow(TrajDuration,4), 		pow(TrajDuration,5),
			 0.0, 1.0, 					2.0*pow(TrajDuration,1), 	3.0*pow(TrajDuration,2), 	4.0*pow(TrajDuration,3), 	5.0*pow(TrajDuration,4),
			 0.0, 0.0, 					2.0, 						6.0*pow(TrajDuration,1), 	12.0*pow(TrajDuration,2),	20.0*pow(TrajDuration,3);

	StateVec.row(0) = _StartPos.transpose();
	StateVec.row(1) = _StartVel.transpose();
	StateVec.row(3) = _FinalPos.transpose();

	Coefficient.noalias() += m_cof.inverse()*StateVec;
	m_isReady = 1;

}

void Trajectory::Poly5th( const double &_CurrentTime, Eigen::VectorXd &_dq, Eigen::VectorXd &_dqdot, Eigen::VectorXd &_dqddot )
{
	if( (_CurrentTime - TrajInitTime) >= TrajDuration )
	{
		m_isReady = 0;

		_dq = StateVec.row(3).transpose(); // Final Position
		_dqdot = StateVec.row(4).transpose();
		_dqddot = StateVec.row(5).transpose();

	}
	else if( m_isReady )
	{
		TrajTime = _CurrentTime - TrajInitTime;

		_dq.setZero();
		_dqdot.setZero();
		_dqddot.setZero();

		for(int i=0; i<NumberJoint; i++)
		{
			for(int j=0; j<6; j++)
			{
				_dq(i) += pow(TrajTime, j)*Coefficient(j,i);
				if(j>=1)
					_dqdot(i) += j*pow(TrajTime, j-1)*Coefficient(j,i);
				if(j>=2)
					_dqddot(i) += j*(j-1)*pow(TrajTime, j-2)*Coefficient(j,i);
			}
		}
	}
	return;
}

} /* namespace HYUDA */


