//
// Created by junho on 21. 6. 23..
//

#include "slerpHandler.h"

slerpHandler::slerpHandler()
{

}

slerpHandler::~slerpHandler()
{

}

void slerpHandler::slerp_setup(Matrix3d rot_a, Matrix3d rot_d, double current_time, double blending_time)
{
    TrajDuration = blending_time;
    TrajInitTime = current_time;

    Coefficient.setZero(6,1);
    StateVec.setZero(6,1);

    m_cof << 1.0, 0.0, 					0.0, 						0.0, 						0.0, 						0.0,
            0.0, 1.0, 					0.0, 						0.0, 						0.0, 						0.0,
            0.0, 0.0, 					2.0, 						0.0, 						0.0, 						0.0,
            1.0, pow(TrajDuration,1), 	pow(TrajDuration,2), 		pow(TrajDuration,3), 		pow(TrajDuration,4), 		pow(TrajDuration,5),
            0.0, 1.0, 					2.0*pow(TrajDuration,1), 	3.0*pow(TrajDuration,2), 	4.0*pow(TrajDuration,3), 	5.0*pow(TrajDuration,4),
            0.0, 0.0, 					2.0, 						6.0*pow(TrajDuration,1), 	12.0*pow(TrajDuration,2),	20.0*pow(TrajDuration,3);

    StateVec(0) = 0.0;
    StateVec(3) = 1.0;

    Coefficient.noalias() += m_cof.inverse()*StateVec;

    q_init = rot_a;
    q_final = rot_d;

    q_log = log_q(q_init.inverse()*q_final);
}


void slerpHandler::slerp_profile(Vector3d &rddot, Vector3d &rdot, Quaterniond &r, double &current_t)
{
    rdot.setZero();
    rddot.setZero();

    if( (current_t - TrajInitTime) >= TrajDuration )
    {
        r = q_final;
        rdot.setZero();
        rddot.setZero();
    }
    else
    {
        TrajTime = current_t - TrajInitTime;
        t_traj=0.0;
        t_traj_dot=0.0;
        t_traj_ddot=0.0;
        for(int j=0; j<6; j++)
        {
            t_traj += pow(TrajTime, j)*Coefficient(j);

            if(j>=1)
            {
                t_traj_dot += j*pow(TrajTime, j-1)*Coefficient(j);
            }
            if(j>=2)
            {
                t_traj_ddot += j*(j-1)*pow(TrajTime, j-2)*Coefficient(j);
            }

        }

        //r = slerp_legacy(q_init, q_final, t_traj);
        r = slerp_gael(q_init, q_final, t_traj);
        rdot = t_traj_dot*q_log.vec();
        rddot = t_traj_ddot*q_log.vec();
    }
}

double slerpHandler::sin_over_x(double x) {
    if(1.0 + x*x == 1.0)
        return 1.0;
    else
        return std::sin(x)/x;
}


Quaterniond slerpHandler::slerp_eigen(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    if(alpha >= 1.0)
        return q1;
    else
        return q0.slerp(alpha,q1);
}

Quaterniond slerpHandler::slerp_legacy(Quaterniond &q0, Quaterniond &q1, double alpha)
{

    if(alpha >= 1.0)
        return q1;

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

Quaterniond slerpHandler::slerp_rw(Quaterniond &q0, Quaterniond &q1, double alpha)
{
    if(alpha >= 1.0)
        return q1;

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

Quaterniond slerpHandler::slerp_gael(Quaterniond &q0, Quaterniond &q1, double alpha)
{

    if(alpha >= 1.0)
        return q1;

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

Quaterniond slerpHandler::exp_q(const Eigen::Quaterniond &q)
{
    double a = q.vec().norm();
    double exp_w = exp(q.w());

    if(a == double(0))
    {
        return Quaterniond(exp_w, 0, 0, 0);
    }

    Quaterniond res;
    res.w() = exp_w*double(cos(a));
    res.vec() = exp_w * double(sin(a))*q.vec();

    return res;
}

Quaterniond slerpHandler::log_q(const Eigen::Quaterniond &q)
{
    double exp_w = q.norm();
    double w = log(exp_w);
    double a = acos(q.w() / exp_w);

    if(a == double(0))
    {
        return Quaterniond(w, 0, 0, 0);
    }

    Quaterniond res;
    res.w() = w;
    res.vec() = q.vec()/exp_w/(sin(a)/a);

    return res;
}