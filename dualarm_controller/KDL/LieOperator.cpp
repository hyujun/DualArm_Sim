/*
 * LieOperator.cpp
 *
 *  Created on: Aug 13, 2018
 *      Author: Junho Park
 */

#include "LieOperator.h"

namespace HYUMotionBase {

    LieOperator::LieOperator() {

    }

    LieOperator::~LieOperator() {

    }

    SE3 LieOperator::inverse_SE3( const SE3 &_SE3 )
    {
        SE3Res.setZero();
        SE3Res << _SE3.block<3, 3>(0, 0).transpose(), -_SE3.block<3, 3>(0, 0).transpose()*_SE3.block<3, 1>(0, 3),
                0, 0, 0, 1;
        return SE3Res;
    }

    void LieOperator::inverse_SE3( const SE3 &_SE3, SE3 &_TargetSE3 )
    {
        _TargetSE3 << _SE3.block<3, 3>(0, 0).transpose(), -_SE3.block<3, 3>(0, 0).transpose()*_SE3.block<3, 1>(0, 3),
                0, 0, 0, 1;
        return;
    }


    Matrix3d LieOperator::SkewMatrix( const Vector3d &_Vec3 )
    {
        Mat3dRes << 0, 		-_Vec3(2), 	_Vec3(1),
                _Vec3(2), 	0, 			-_Vec3(0),
                -_Vec3(1), 	_Vec3(0), 	0;
        return Mat3dRes;
    }

    void LieOperator::SkewMatrix( const Vector3d &_Vec3, Matrix3d &_TargetMat )
    {
        _TargetMat << 0, -_Vec3(2), _Vec3(1),
                _Vec3(2), 0, -_Vec3(0),
                -_Vec3(1), _Vec3(0), 0;
        return;
    }

    Vector3d LieOperator::InvSkewMatrix( const Matrix3d &_Mat3 )
    {
        Vec3dRes(0) = -_Mat3(0,1);
        Vec3dRes(1) = _Mat3(0,2);
        Vec3dRes(2) = -_Mat3(1,2);
        return Vec3dRes;
    }

    Matrix3d LieOperator::SkewMatrixSquare( const Vector3d &_Vec3 )
    {
        Mat3dRes << -_Vec3(2)*_Vec3(2)-_Vec3(1)*_Vec3(1), 	_Vec3(0)*_Vec3(1), 						_Vec3(0)*_Vec3(2),
                _Vec3(0)*_Vec3(1), 						-_Vec3(0)*_Vec3(0)-_Vec3(2)*_Vec3(2), 	_Vec3(1)*_Vec3(2),
                _Vec3(0)*_Vec3(2), 						_Vec3(1)*_Vec3(2), 						-_Vec3(0)*_Vec3(0)-_Vec3(1)*_Vec3(1);
        return Mat3dRes;
    }

    void LieOperator::SkewMatrixSquare(const Vector3d &_Vec3, Matrix3d &_TargetMat)
    {
        _TargetMat << -_Vec3(2)*_Vec3(2) - _Vec3(1)*_Vec3(1), _Vec3(0)*_Vec3(1), _Vec3(0)*_Vec3(2),
                _Vec3(0)*_Vec3(1), -_Vec3(0)*_Vec3(0) - _Vec3(2)*_Vec3(2), _Vec3(1)*_Vec3(2),
                _Vec3(0)*_Vec3(2), _Vec3(1)*_Vec3(2), -_Vec3(0)*_Vec3(0) - _Vec3(1)*_Vec3(1);
        return;
    }

    Adjoint LieOperator::AdjointMatrix( const SE3 &_SE3 )
    {
        AdjointRes.setZero();

        AdjointRes.block<3,3>(0,0) = _SE3.block(0, 0, 3, 3);
        AdjointRes.block<3,3>(3,3) = _SE3.block(0, 0, 3, 3);
        AdjointRes.block<3,3>(3,0).noalias() += SkewMatrix(_SE3.block(0, 3, 3, 1))*_SE3.block(0, 0, 3, 3);

        return AdjointRes;
    }

    void LieOperator::AdjointMatrix( const SE3 &_SE3, Adjoint &_TargetAdjoint )
    {
        _TargetAdjoint.setZero();

        _TargetAdjoint.block<3, 3>(0, 0) = _SE3.block(0, 0, 3, 3);
        _TargetAdjoint.block<3, 3>(3, 3) = _SE3.block(0, 0, 3, 3);
        _TargetAdjoint.block<3, 3>(3, 0).noalias() += SkewMatrix(_SE3.block(0, 3, 3, 1))*_SE3.block(0, 0, 3, 3);

        return;
    }

    Adjoint LieOperator::AdjointDualMatrix( const SE3 &_SE3 )
    {
        AdjointRes.setZero();

        AdjointRes.block(0, 0, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
        AdjointRes.block(0, 3, 3, 3).noalias() += _SE3.block(0, 0, 3, 3).transpose()*SkewMatrix(_SE3.block(0, 3, 3, 1)).transpose();
        AdjointRes.block(3, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
        return AdjointRes;
    }

    void LieOperator::AdjointDualMatrix( const SE3 &_SE3, Adjoint &_TargetAdjoint )
    {
        _TargetAdjoint.setZero();

        _TargetAdjoint.block(0, 0, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
        _TargetAdjoint.block(0, 3, 3, 3).noalias() += _SE3.block(0, 0, 3, 3).transpose()*SkewMatrix(_SE3.block(0, 3, 3, 1)).transpose();
        _TargetAdjoint.block(3, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
        return;
    }

    adjoint LieOperator::adjointMatrix( const se3 &_se3 )
    {
        adjointRes.setZero();

        adjointRes.block(0, 0, 3, 3) = SkewMatrix(_se3.head(3));
        adjointRes.block(3, 0, 3, 3) = SkewMatrix(_se3.tail(3));
        adjointRes.block(3, 3, 3, 3) = SkewMatrix(_se3.head(3));
        return adjointRes;
    }

    void LieOperator::adjointMatrix( const se3 &_se3, adjoint &_Targetadjoint )
    {
        _Targetadjoint.setZero();

        _Targetadjoint.block(0, 0, 3, 3) = SkewMatrix(_se3.head(3));
        _Targetadjoint.block(3, 0, 3, 3) = SkewMatrix(_se3.tail(3));
        _Targetadjoint.block(3, 3, 3, 3) = SkewMatrix(_se3.head(3));
        return;
    }

    adjoint LieOperator::adjointDualMatrix( const se3 &_se3 )
    {
        adjointRes.setZero();

        adjointRes.block(0, 0, 3, 3) = -SkewMatrix(_se3.head(3));
        adjointRes.block(0, 3, 3, 3) = -SkewMatrix(_se3.tail(3));
        adjointRes.block(3, 3, 3, 3) = -SkewMatrix(_se3.head(3));

        return adjointRes;
    }

    void LieOperator::adjointDualMatrix( const se3 &_se3, adjoint &_Targetadjoint )
    {
        _Targetadjoint.setZero();

        _Targetadjoint.block(0, 0, 3, 3) = -SkewMatrix(_se3.head(3));
        _Targetadjoint.block(0, 3, 3, 3) = -SkewMatrix(_se3.tail(3));
        _Targetadjoint.block(3, 3, 3, 3) = -SkewMatrix(_se3.head(3));

        return;
    }

    SO3 LieOperator::ExpSO3Matrix( const Vector3d &_omega, const double &_theta)
    {
        SO3Res.setIdentity();
        SO3Res.noalias() += sin(_theta)*SkewMatrix(_omega);
        SO3Res.noalias() += (1 - cos(_theta))*SkewMatrixSquare(_omega);
        return SO3Res;
    }

    void LieOperator::LogSO3( const SO3 &_RotMat, Vector3d &_omega, double &_theta )
    {
        mLogSO3 = _RotMat;
        _omega = mLogSO3.axis();
        _theta = mLogSO3.angle();
        return;
    }

    Matrix3d LieOperator::GmapMatrix( const Vector3d &_omega, const double &_theta )
    {
        Mat3dRes.setZero();
        Mat3dRes.noalias() += Matrix3d::Identity()*_theta;
        Mat3dRes.noalias() += (1 - cos(_theta))*SkewMatrix(_omega);
        Mat3dRes.noalias() += (_theta - sin(_theta))*SkewMatrixSquare(_omega);
        return Mat3dRes;
    }

    Matrix3d LieOperator::GinvmapMatrix( const Vector3d &_omega, const double &_theta )
    {
        return Mat3dRes;
    }



    SE3 LieOperator::SE3Matrix( const se3 &_Twist, const double &_q )
    {
        Matrix3d i = Matrix3d::Identity();
        Mat4dRes.setZero();
        Mat4dRes.block(0, 0, 3, 3) = i;
        Mat4dRes.block(0, 0, 3, 3).noalias() += sin(_q)*SkewMatrix(_Twist.head(3));
        Mat4dRes.block(0, 0, 3, 3).noalias() += (1 - cos(_q))*SkewMatrixSquare(_Twist.head(3));
        Mat4dRes.block(0, 3, 3, 1).noalias() += ((i*_q) + ((1 - cos(_q))*SkewMatrix(_Twist.head(3))) + ((_q - sin(_q))*SkewMatrixSquare(_Twist.head(3))))*_Twist.tail(3);
        Mat4dRes.block(3, 0, 1, 4) << 0, 0, 0, 1;

        return Mat4dRes;
    }

    void LieOperator::SE3Matrix( const se3 &_Twist, const double &_q, SE3 &_TargetSE3 )
    {
        Matrix3d i = Matrix3d::Identity();
        _TargetSE3.setZero();
        _TargetSE3.block(0, 0, 3, 3) = i;
        _TargetSE3.block(0, 0, 3, 3).noalias() += sin(_q)*SkewMatrix(_Twist.head(3));
        _TargetSE3.block(0, 0, 3, 3).noalias() += (1 - cos(_q))*SkewMatrixSquare(_Twist.head(3));
        _TargetSE3.block(0, 3, 3, 1).noalias() += ((i*_q) + ((1 - cos(_q))*SkewMatrix(_Twist.head(3))) + ((_q - sin(_q))*SkewMatrixSquare(_Twist.head(3))))*_Twist.tail(3);
        _TargetSE3.block(3, 0, 1, 4) << 0, 0, 0, 1;

        return;
    }

    void LieOperator::invExpdExpInvMapMatrix( const Vector3d &_omega, const double &_theta, MatrixXd &_dexpinv )
    {
        _dexpinv.resize(6,6);
        _dexpinv.setZero();
        _dexpinv.block(3,3, 3,3) = ExpSO3Matrix(_omega, _theta);

        if(abs(_theta) <= 1e-8)
        {
            _dexpinv.block(0,0,3,3) = Matrix3d::Identity();
        }
        else
        {
            _dexpinv.block(0,0,3,3) += Matrix3d::Identity();
            _dexpinv.block(0,0,3,3) += 1/2*SkewMatrix(_omega*_theta);
            _dexpinv.block(0,0,3,3) += ((1/pow(_theta,2) - (1-cos(_theta)/(2*_theta*sin(_theta))))*SkewMatrixSquare(_omega*_theta));
        }

        return;
    }

    void LieOperator::invExpdExpMapMatrix( const Vector3d &_omega, const double &_theta, MatrixXd &_dexp )
    {
        Vector3d r;
        _dexp.resize(6,6);
        _dexp.setZero();
        _dexp.block(3,3, 3,3) = ExpSO3Matrix(_omega, _theta).transpose();

        r = _omega*_theta;

        if(abs(_theta) <= 1e-8)
        {
            _dexp.block(0,0,3,3) = Matrix3d::Identity();
        }
        else
        {
            _dexp.block(0,0,3,3) += Matrix3d::Identity();
            _dexp.block(0,0,3,3) += -(1-cos(r.norm()))/r.squaredNorm()*SkewMatrix(r);
            _dexp.block(0,0,3,3) += (r.norm() - sin(r.norm()))/(pow(r.norm(), 3))*SkewMatrixSquare(r);
        }

        return;
    }

} /* namespace HYUMotion */
