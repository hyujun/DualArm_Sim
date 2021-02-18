/**
 * @file LieOperator.h
 * @author Junho Park
 */

#ifndef LIEOPERATOR_H_
#define LIEOPERATOR_H_

#include <Eigen/Dense>
using namespace Eigen;

typedef Matrix<double, 6, 1> se3;		/**<Size definition of se3(vector)*/
typedef Matrix<double, 4, 4> SE3;		/**<Size definition of SE3(matrix)*/
typedef Matrix<double, 6, 6> Adjoint;		/**<Size definition of Adjoint operator(twist)*/
typedef Matrix<double, 6, 6> adjoint;		/**<Size definition of adjoint operator(wrench)*/
typedef Matrix3d SO3;

/**
 * @brief Biorobotics Lab Lie-Group Operator
 * @date 2018-08-13
 * @version 1.0.0
 */
namespace HYUMotionBase {

/**
 * @brief Lie-algebra operator collection
 * @see ModernRobotics book
 */
class LieOperator {
public:
	LieOperator();
	virtual ~LieOperator();

public:
	/**
	 * @brief Inverse of Homogeneous Transform Matrix SE(3)
	 * @param[in] _SE3 Homogeneous Transform Matrix SE(3)
	 * @return SE3
	 */
	SE3 inverse_SE3( const SE3 &_SE3 );
	void inverse_SE3( const SE3 &_SE3, SE3 &_TargetSE3 );

	/**
	 * @brief Vector to Skew-Symmetric Matrix SO(3)
	 * @param[in] _Vec3 vector
	 * @return SO(3) skew-symmetric matrix
	 */
	Matrix3d SkewMatrix( const Vector3d &_Vec3 );
	void SkewMatrix( const Vector3d &_Vec3, Matrix3d &_TargetMat );

	Vector3d InvSkewMatrix( const Matrix3d &_Mat3 );

	/**
	 * @brief Square of skew-symmetric matrix
	 * @param[in] _Vec3 vector
	 * @return SO(3) square of skew-symmetric matrix
	 */
	Matrix3d SkewMatrixSquare( const Vector3d &_Vec3 );
	void SkewMatrixSquare( const Vector3d &_Vec3, Matrix3d &_TargetMat );

	/**
	 * @brief Make SE(3) to Adjoint-Operator(twist)
	 * @param[in] _SE3
	 * @return Adjoint Matrix(twist)
	 */
	Adjoint AdjointMatrix( const SE3 &_SE3 );
	void AdjointMatrix( const SE3 &_SE3, Adjoint &_TargetAdjoint );

	/**
	 * @brief Make SE(3) to Dual-Adjoint-Operator(twist)
	 * @param[in] _SE3
	 * @return dual-Adjoint matrix (twist)
	 */
	Adjoint AdjointDualMatrix( const SE3 &_SE3 );
	void AdjointDualMatrix( const SE3 &_SE3, Adjoint &_TargetAdjoint );

	/**
	 * @brief Make se(3) to adjoint-Operator(wrench)
	 * @param[in] _se3
	 * @return adjoint matrix (wrench)
	 */
	adjoint adjointMatrix( const se3 &_se3 );
	void adjointMatrix(const se3 &_se3, adjoint &_Targetadjoint);

	/**
	 * @brief Make se(3) to dual adjoint-Operator(wrench)
	 * @param[in] _se3
	 * @return dual adjoint matrix (wrench)
	 */
	adjoint adjointDualMatrix( const se3 &_se3 );
	void adjointDualMatrix(const se3 &_se3, adjoint &_Targetadjoint);

	SO3 ExpSO3Matrix( const Vector3d &_omega, const double &_theta);
	void LogSO3( const SO3 &_RotMat, Vector3d &_omega, double &_theta );

	Matrix3d GmapMatrix( const Vector3d &_omega, const double &_theta );
	Matrix3d GinvmapMatrix( const Vector3d &_omega, const double &_theta );

	void invExpdExpMapMatrix( const Vector3d &_omega, const double &_theta, MatrixXd &_dexp );
	void invExpdExpInvMapMatrix( const Vector3d &_omega, const double &_theta, MatrixXd &_dexpinv );

	/**
	 * @brief Make Homogeneous transformation matrix
	 * @param[in] _Twist
	 * @param[in] _q generalized coordinate joint position
	 * @return Homogeneous transformation matrix SE(3)
	 */
	SE3 SE3Matrix( const se3 &_Twist, const double &_q );
	void SE3Matrix( const se3 &_Twist, const double &_q, SE3 &_TargetSE3 );

private:

	SE3 SE3Res;
	se3 se3Res;
	SO3 SO3Res;
	Vector3d Vec3dRes;
	Adjoint AdjointRes;
	adjoint adjointRes;
	Matrix3d Mat3dRes;
	Matrix4d Mat4dRes;

	AngleAxisd mLogSO3;

};

} /* namespace HYUMotion */

#endif /* LIEOPERATOR_H_ */
