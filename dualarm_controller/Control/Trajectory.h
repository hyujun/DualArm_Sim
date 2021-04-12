/**
 * @file Trajectory.h
 * @date 2018-11-23
 * @author: Junho Park
 */

#ifndef CONTROL_TRAJECTORY_H_
#define CONTROL_TRAJECTORY_H_

#include <Eigen/Dense>

#define _USE_MATH_DEFINES
#include <cmath>

namespace HYUControl {

/**
 * @brief Trajectory Class is 5th order polynomial trajectory generator interface
 * @version 1.1.0
 */
class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	/**
	 * @brief Confirm the TrajectoryClass is ready
	 * @return 1 : ready \n
	 * 0 : not ready
	 */
	bool isReady(void){return m_isReady;};

	/**
	 * @brief set the 5th polynomial trajectory
	 * @param[in] _StartPos start joint position
	 * @param[in] _FinalPos final joint position
	 * @param[in] _InitTime start time
	 * @param[in] _Duration trajectory time
	 */
	void SetPoly5th( const double &_CurrentTime, const Eigen::VectorXd &_StartPos, const Eigen::VectorXd &_StartVel, const Eigen::VectorXd &_FinalPos, double &_Duration, int &_NumJoint );

	/**
	 * @brief generate the trajecotry for q, qdot, qddot
	 * @param[in] _CurrentTime current time in second
	 * @param[in] _dq returns value for desired q
	 * @param[in] _dqdot returns value for desired qdot
	 * @param[in] _dqddot returns value for desired qddot
	 */
	void Poly5th( const double &_CurrentTime, Eigen::VectorXd &_dq, Eigen::VectorXd &_dqdot, Eigen::VectorXd &_dqddot );


private:

	int m_isReady;

	Eigen::Matrix<double, 6, 6> m_cof;
	Eigen::MatrixXd StateVec, Coefficient;

	double TrajTime=0;
	double TrajDuration=0;
	double TrajInitTime=0;
	int NumberJoint=0;

};

} /* namespace HYUDA */

#endif /* CONTROL_TRAJECTORY_H_ */
