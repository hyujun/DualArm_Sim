//
// Created by junho on 21. 6. 23..
//

#ifndef SRC_SLERP_H
#define SRC_SLERP_H

#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

class slerp {

public:
    slerp();
    virtual ~slerp();


    int slerp_isready();
    void slerp_setup(Matrix3d rot_a, Matrix3d rot_d, double current_time, double blending_time);
    void slerp_profile(Vector3d rddot, Vector3d rdot, Vector3d r, double current_t);


private:
    Quaterniond slerp_eigen(Quaterniond& q0, Quaterniond& q1, double alpha);
    Quaterniond slerp_legacy(Quaterniond& q0, Quaterniond& q1, double alpha);
    Quaterniond slerp_rw(Quaterniond& q0, Quaterniond& q1, double alpha);
    Quaterniond slerp_gael(Quaterniond& q0, Quaterniond& q1, double alpha);

    double sin_over_x(double x);

    double TrajDuration;
    double TrajInitTime;
    double TrajTime=0;
    double t_traj;
    Eigen::Matrix<double, 6, 6> m_cof;
    Eigen::VectorXd StateVec, Coefficient;

    int m_isReady=0;

    Quaterniond q_init;
    Quaterniond q_final;
};




#endif //SRC_SLERP_H
