//
// Created by junho on 21. 6. 23..
//

#include "slerp.h"


void slerp::slerp_setup(Matrix3d rot_a, Matrix3d rot_d, double current_time, double blending_time)
{
    TrajDuration = blending_time;
    TrajInitTime = current_time;

    Coefficient.setZero(6);
    StateVec.setZero(6);

    m_cof << 1.0, 0.0, 					0.0, 						0.0, 						0.0, 						0.0,
            0.0, 1.0, 					0.0, 						0.0, 						0.0, 						0.0,
            0.0, 0.0, 					2.0, 						0.0, 						0.0, 						0.0,
            1.0, pow(TrajDuration,1), 	pow(TrajDuration,2), 		pow(TrajDuration,3), 		pow(TrajDuration,4), 		pow(TrajDuration,5),
            0.0, 1.0, 					2.0*pow(TrajDuration,1), 	3.0*pow(TrajDuration,2), 	4.0*pow(TrajDuration,3), 	5.0*pow(TrajDuration,4),
            0.0, 0.0, 					2.0, 						6.0*pow(TrajDuration,1), 	12.0*pow(TrajDuration,2),	20.0*pow(TrajDuration,3);

    StateVec(0) = 0;
    StateVec(1) = 0;
    StateVec(3) = 1;

    Coefficient.noalias() += m_cof.inverse()*StateVec;

    q_init = rot_a;
    q_final = rot_d;

    m_isReady = 1;
}


void slerp::slerp_profile(Vector3d rddot, Vector3d rdot, Vector3d r, double current_t)
{
    if( (current_t - TrajInitTime) >= TrajDuration )
    {
        m_isReady = 0;

        r = q_final.vec(); // Final Position
        rdot.setZero();
        rddot.setZero();

    }
    else if( m_isReady )
    {
        TrajTime = current_t - TrajInitTime;

        for(int j=0; j<6; j++)
        {
            t_traj += pow(TrajTime, j)*Coefficient(j,0);
        }

        Quaterniond qtar = slerp_eigen(q_init, q_final, t_traj);
        r = qtar.vec();

        Quaterniond qlog = q_init.inverse()*q_final;
        rdot = qtar.vec()*((1.0/qlog.vec().norm())*acos(qlog.w()/qlog.norm()));

        AngleAxisd angleaxis;
        angleaxis = qlog;
        rddot = qtar.vec()*(-angleaxis.angle()*angleaxis.angle());
    }
}

double slerp::sin_over_x(double x) {
    if(1.0 + x*x == 1.0)
        return 1.0;
    else
        return std::sin(x)/x;
}


Quaterniond slerp::slerp_eigen(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    return q0.slerp(alpha,q1);
}

Quaterniond slerp::slerp_legacy(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    double one = 1.0 - NumTraits<double>::dummy_precision();
    double d = q0.dot(q1);
    double absD = abs(d);

    if(absD >= one)
        return q0;

    double theta = std::acos(absD);
    double sinTheta = sin(theta);

    double scale0 = sin( (1.0 - alpha)*theta ) / sinTheta;
    double scale1 = sin( (alpha*theta) )/sinTheta;

    if(d<0)
        scale1 = -scale1;

    return Quaterniond(scale0*q0.coeffs() + scale1*q1.coeffs());
}

Quaterniond slerp::slerp_rw(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    double d = q0.dot(q1);
    double theta;

    if(d < 0.0)
        theta = 2.0*std::asin( (q0.coeffs() + q1.coeffs()).norm()/2 );
    else
        theta = 2.0*std::asin( (q0.coeffs() - q1.coeffs()).norm()/2 );

    double sinOverTheta = sin_over_x(theta);

    double scale0 = ( 1.0-alpha )*sin_over_x( ( 1.0-alpha )*theta )/sinOverTheta;
    double scale1 = alpha*sin_over_x( (alpha*theta) )/sinOverTheta;

    if(d < 0.0)
        scale1 = -scale1;

    return Quaterniond(scale0*q0.coeffs() + scale1*q1.coeffs());
}

Quaterniond slerp::slerp_gael(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    double d = q0.dot(q1);
    double theta;

    if(d < 0.0)
        theta = 2.0*std::asin( (-q0.coeffs() - q1.coeffs()).norm()/2 );
    else
        theta = 2.0*std::asin( (q0.coeffs() - q1.coeffs()).norm()/2 );

    double scale0;
    double scale1;

    if(theta*theta - 6.0 == -6.0)
    {
        scale0 = 1.0 - alpha;
        scale1 = alpha;
    }
    else
    {
        double sinTheta = std::sin(theta);
        scale0 = sin( (1.0-alpha)*theta )/sinTheta;
        scale1 = sin( (alpha*theta) )/sinTheta;
        if(d<0)
            scale1 = -scale1;
    }

    return Quaterniond( scale0*q0.coeffs() + scale1*q1.coeffs() );
}

