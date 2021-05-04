/**
 * @file PoEKinematics.h
 * @brief Product of Exponential formulation for Kinematics
 * @date 2019-09-17
 * @author Junho Park
 */

#ifndef POEKINEMATICS_H_
#define POEKINEMATICS_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include "LieOperator.h"
#include "PropertyDefinition.h"

/**
 * @brief [Biorobotics Lab] Kinematics Solver using Lie-Group(Differential Kinematics)
 * @version 1.2.0
 */
namespace HYUMotionKinematics {

/**
 * @brief PoEKinematics Class for Tree-type Manipulator
 * @version 1.2.0
 */
    class PoEKinematics : public HYUMotionBase::LieOperator {
    public:
        /**
         * @brief PoEKinematics class constructor
         * @details A chain matrix should be defined.
         */
        PoEKinematics();
        /**
         * @brief PoEKinematics class constructor
         * @details A chain matrix should be defined.
         */
        PoEKinematics( const MatrixXi &_ChainMat );
        virtual ~PoEKinematics();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * @brief Construct the kinematic infomation
         * @param[in] _w omega(twist)
         * @param[in] _p link position
         * @param[in] _l link length
         * @param[in] _link_num number of link attached to base-coordinate
         */
        void UpdateKinematicInfo( const Vector3d &_w, const Vector3d &_p, const Vector3d &_Rot, const Vector3d &_l, const int _link_num );

        /**
         * @brief Calculate the joint velocity v
         * @param[in] _w joint axis with respect to the base coordinate
         * @param[in] _p lint position attached to joint coordinate
         * @return v
         */
        Vector3d GetV( const Vector3d &_w, const Vector3d &_p );

        /**
         * @brief Calculate the initial configuration of serial robot
         * @param[in] _link total length of robot
         * @return SE(3)
         */
        SE3 GetM( const Vector3d &_Rot, const Vector3d &_link );

        void SetTwist( const se3 &_Twist, const int _link_num );
        /**
         * @brief Calculate the Twist of joint
         * @param[in] _w joint axis with respect to the base coordinate
         * @param[in] _v joint velocity
         * @return se3 vector
         */
        se3 GetTwist( const Vector3d &_w, const Vector3d &_v );

        /**
         * @brief Calculate the Homogeneous transformation matrix SE(3)
         * @param[in] _q generalized coordinate of joint position
         */
        void HTransMatrix( const VectorXd &_q );

        void PrepareJacobian( const VectorXd &_q );
        /**
         * @brief calculate the space jacobian
         * @return 6 x n(DoF) jacobian matrix w.r.t, base coordinate
         */
        void GetSpaceJacobian( MatrixXd &_SpaceJacobian )
        {
            _SpaceJacobian = mSpaceJacobian;
        }

        /**
         * @brief calculate the body jacobian
         * @return 6 x n(DoF) jacobian matrix w.r.t., end-effector coordinate
         */
        void GetBodyJacobian( MatrixXd &_BodyJacobian )
        {
            _BodyJacobian = mBodyJacobian;
        }

        void GetBodyJacobianDot( MatrixXd &_BodyJacobianDot );

        /**
         * @brief calcuate the analytic jacobian
         * @return 6 x n(DoF) jacobian matrix
         */
        void GetAnalyticJacobian( MatrixXd &_AnalyticJacobian )
        {
            _AnalyticJacobian = mAnalyticJacobian;
        }

        void GetAnalyticJacobianDot(const VectorXd &_qdot, MatrixXd &_AnalyticJacobianDot);

        void GetpinvJacobian( MatrixXd &_pinvJacobian );

        void GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian );

        void GetDampedpInvJacobian( MatrixXd &_DampedpInvJacobian );

        void GetDampedpInvJacobian( MatrixXd &_TargetMat, MatrixXd &_DampedpInvJacobian );

        void GetBlockpInvJacobian( MatrixXd &_BlockpInvJacobian );

        void GetRelativeJacobian( MatrixXd &_RelativeJacobian );

        void GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_WDampedpInvJacobian );

        void GetWDampedpInvLambda(VectorXd *lambda);

        void GetInverseConditionNumber( double *_InverseCondNumber );

        double GetManipulabilityMeasure();

        void Getq0dotWithMM(const double &gain, VectorXd &q0dot);
        /**
         * @brief forward kinematics of serial robot
         * @return end-effector position x, y, z. not orientation(Working)
         */
        void GetForwardKinematics( Vector3d *_Position, Vector3d *_Orientation, int &_NumChain );

        SE3 GetForwardKinematicsSE3( const int &_EndPosition ) const;

        SO3 GetForwardKinematicsSO3( const int &_EndPosition ) const;

        void GetAngleAxis( Vector3d *_Axis, double *_Angle, int &_NumChain );

        void SO3toAngleAxis( const Matrix3d &_RotMat, Vector3d &_orientation );

        void SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation );

        void RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat);

        SE3 GetTMat(const int _begin, const int _end)
        {
            return T[_begin][_end];
        }

        int GetNumChain() const
        {
            return m_NumChain;
        }

        se3 GetTwist(const int _pos) const
        {
            return v_se3[_pos];
        }

        SE3 GetMMat(const int _pos) const
        {
            return M[_pos];
        }

    protected:

        void SpaceJacobian();

        void SpaceToBodyJacobian();

        void BodyJacobianDot( const VectorXd &_qdot );

        void AnalyticJacobian();

        void AnalyticJacobianDot( const VectorXd &_qdot );

        void ScaledTransJacobian();

        void pInvJacobian();

        void DampedpInvJacobian(const double sigma);

        void DampedpInvJacobian( MatrixXd &_TargetMatrix, const double sigma);

        void RelativeJacobian(const int From, const int To);

        void BlockpInvJacobian( Matrix<double, 6, Dynamic> &_Jacobian1, Matrix<double, 6, Dynamic> &_Jacobian2 );

        void WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat );

        MatrixXi ChainMatrix;
        int m_NumChain;
        int m_DoF;
        int ChainJointCount[2];
        int JointEndNum[2];

        SE3 SE3_Tmp;
        se3 se3_Tmp;

        VectorXi Arr[2];

        MatrixXd mSpaceJacobian;
        MatrixXd mBodyJacobian;
        MatrixXd mBodyJacobianDot;
        MatrixXd mAnalyticJacobian;
        MatrixXd mAnalyticJacobianDot;
        VectorXd ScaledFactor;
        MatrixXd mScaledTransJacobian;
        MatrixXd mpInvJacobin;
        MatrixXd mDampedpInvJacobian;
        MatrixXd mBlockpInvJacobian;
        MatrixXd mWeightDampedpInvJacobian;
        Eigen::Matrix<double, 6, Dynamic> mRelativeJacobian;

        MatrixXd Mat_Tmp;
        VectorXd Vec_Tmp;

        Quaterniond q;

        double WpInv_epsilon_left;
        double WpInv_epsilon_right;
        VectorXd lambda_left;
        VectorXd lambda_right;

        /**
         * @brief SE(3) Homogeneous transform matrix container
         */
        SE3 T[17][17];
        //SE3 **T;

        /**
         * @brief SE(3) matrix container w.r.t., base coordinate
         */
        SE3 M[17];
        //SE3 *M;

        /**
         * @brief SE(3) matrix container
         */
        SE3 Exp_S[17];
        //SE3 *Exp_S;

        /**
         * @brief twist expression for Adjoint/adjoint matrix
         */
        se3 v_se3[17];
        //se3 *v_se3;

    };

}

#endif /* POEKINEMATICS_H_ */
