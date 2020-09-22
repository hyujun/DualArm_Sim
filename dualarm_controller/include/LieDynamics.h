#pragma once

/**
 * @file LieDynamics.h
 * @date 2019-09-17
 * @author Junho Park
 */

#define _USE_MATH_DEFINED
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SVD>
using namespace Eigen;

#include "PropertyDefinition.h"
#include "PoEKinematics.h"

#include <iostream>
using namespace std;

typedef Matrix<double, 6, 6> Matrix6d;

#include <omp.h>

/**
 * @brief Newton-Euler Dynamics expressed in Lie Group
 * @version 1.1.0
 */
namespace HYUMotionDynamics{

/**
 * @brief [Biorobotics Lab] Dynamics Solver using Lie-Group for Tree-type Manipulator
 * @version 1.2.0
 */
class Liedynamics :public HYUMotionBase::LieOperator //: public HYUMotionKinematics::PoEKinematics
{
public:
	Liedynamics();
	/**
	 * @brief Liedynamics constructor
	 * @param[in] _ChainMatrix
	 * @param[in] _PoEKin
	 */
	Liedynamics( const MatrixXi &_ChainMatrix, HYUMotionKinematics::PoEKinematics &_PoEKin, HYUMotionKinematics::PoEKinematics &_CoMKin );
	~Liedynamics();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	Matrix6d GIner[16]; 	/**< generalized inertia matrix of each link  */
	se3 A[16];
	VectorXd grav;

	MatrixXd Gamma_mat;
	MatrixXd Iner_mat;
	MatrixXd ad_Aqd;
	MatrixXd ad_V;
	MatrixXd L_mat;

	MatrixXd A_mat;
	MatrixXd LA_mat;

	VectorXd V;
	VectorXd VdotBase;

	MatrixXd mM_Tmp;
	MatrixXd mG;

	MatrixXi ChainMatrix;

	HYUMotionKinematics::PoEKinematics *pLink;
	HYUMotionKinematics::PoEKinematics *pCoM;
	MatrixXd Jacobian_mat;
	MatrixXd pinvJacobian_mat;

	int m_NumChain;
	int m_DoF;

	int isFirstRun;

	/**
	 * @brief generalized inertia matrix
	 * @param[in] inertia generalized inertia matrix
	 * @param[in] mass mass value of each link
	 * return generalized inertia matrix Re^3
	 */
	void GeneralizedInertia(const Matrix3d &_Inertia, const double &_Mass, Matrix6d &GIner);

	/**
	 * @brief inertia matrix of total serial robot 6n x 6n
	 * @return 6n x 6n matrix, where n is DoF
	 * @see ModernRobotics book
	 */
	void Inertia_Link(void);


	/**
	 * @brief a part of inertia and coriolis matrix
	 * @return 6n x 6n matrix, where n is DoF
	 */
	void Gamma_Link(void);

	/**
	 * @brief a part of inertia and coriolis matrix
	 * @return 6n x 6n matrix, where n is DoF
	 */
	void L_link(void);

	/**
	 * @brief a part of inertia and coriolis matrix
	 * @return 6n x n matrix, where n is DoF
	 */
	void A_Link(void);

	/**
	 * @brief a part of coriolis matrix
	 * @return 6n x 6n matrix, where n is DoF
	 */
	void ad_V_Link(VectorXd _qdot);

	/**
	 * @brief a part of coriolis matrix
	 * @param[in] _qdot generalized coordinate of joint velocity
	 * @return 6n x 6n matrix, where n is DoF
	 * @see MordernRobotics book
	 */
	void ad_Aqdot_Link(VectorXd _qdot);

	/**
	 * @brief a part of gravity matrix
	 * @param[in] axis gravity acceleration direction w.r.t., base coordinate
	 * @return 6n vector, where n is DoF
	 */
	void Vdot_base(void);

	void Mdot_Matrix( MatrixXd &_Mdot );
public:

	/**
	 * @brief update dynamic infomation
	 * @param[in] _Inertia generalized inertial matrix
	 * @param[in] _Mass mass value of each link
	 * @param[in] _LinkNum number of link from base to end-effector
	 */
	void UpdateDynamicInfo( Matrix3d _Inertia, double _Mass, Vector3d _CoM, int _LinkNum );

	/**
	 * @brief Calculate the basis of dynamics
	 * @param[in] _q generalized coordinate of joint position
	 * @param[in] _qdot generalized coordinate of joint velocity
	 * @see ModernRobotics book
	 */
	void PrepareDynamics( const double *_q, const double *_qdot);

	/**
	 * @brief Coriolis matrix in Motion of Equation
	 */
	void C_Matrix( MatrixXd &_C );

	/**
	 * @brief Gravity matrix in Motion of Equation
	 */
	void G_Matrix( VectorXd &_G );

	/**
	 * @brief Inertia(n x n) & Gravity(n x 1) matrix in Motion of Equation
	 */
	void MG_Mat_Joint( MatrixXd &_M, VectorXd &_G );

	void MG_Mat_Task( const MatrixXd &_M, const VectorXd &_G, MatrixXd &_Mx, VectorXd &_Gx );


};
}

