/*
 * KistHand.cpp
 *
 *  Created on: 2019. 12. 5.
 *      Author: Administrator
 */

#include "KistHand.h"

namespace HYUControl {

KistHand::KistHand() {

	TotalDoF = 2;
	targetpos.resize(TotalDoF);

	dq.resize(TotalDoF);
	dqdot.resize(TotalDoF);
	dqddot.resize(TotalDoF);

	q.resize(TotalDoF);
	qdot.resize(TotalDoF);

	K.resize(TotalDoF);
	K(0) = 0.001;
	K(1) = 0.001;
}

KistHand::~KistHand() {

}


void KistHand::HandEnctoRad( int *_enc, double *_rad )
{
	for(int i=0; i<TotalDoF; i++)
	{
		_rad[i] = (double)(_enc[i])/(66.0*1600.0)*(2.0*M_PI);
	}

}

void KistHand::HandVelocityConvert( int *_enc_s, double *_rad_s )
{
	for(int i=0; i<TotalDoF; i++)
	{
		_rad_s[i] = (double)(_enc_s[i])/(66.0*1600.0)*(2.0*M_PI);
	}
}

void KistHand::HandControl(int _Motion, double *_inc_enc, int32_t *_inc_command_vel, double _Time )
{

	q(0) = _inc_enc[0];
	q(1) = _inc_enc[1];

	HandMotion = _Motion;
	if(HandMotion == 1)
	{
		if(HandMotion_p == HandMotion)
		{
			if(HandJointPoly5th.isReady() == 0 && NewTarget == 1)
			{
				HandJointPoly5th.SetPoly5th(_Time, q, qdot, targetpos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				HandJointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
			}
		}
		else
		{

			IndexFinger[0] = 1*D2R;
			IndexFinger[1] = 1*D2R;

			TrajectoryTime = 5.0;

			targetpos(0) = IndexFinger[0];
			targetpos(1) = IndexFinger[1];


			qdot.setZero();

			NewTarget = 1;
		}
	}
	else if(HandMotion == 2)
	{
		if(HandMotion_p == HandMotion)
		{
			if(HandJointPoly5th.isReady() == 0 && NewTarget == 1)
			{
				HandJointPoly5th.SetPoly5th(_Time, q, qdot, targetpos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
			{
				HandJointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
			}
		}
		else
		{
			IndexFinger[0] = 10.0*D2R;
			IndexFinger[1] = 10.0*D2R;


			TrajectoryTime = 5.0;

			targetpos(0) = IndexFinger[0];
			targetpos(1) = IndexFinger[1];

			qdot.setZero();
			NewTarget = 1;
		}
	}

	if(HandJointPoly5th.isReady() == 1)
		qdot = dqdot + K.cwiseProduct(dq - q);
	else
		qdot.setZero();


	_inc_command_vel[0] = (int32_t)round(qdot(0)/(2.0*M_PI)*(66.0*1600.0));
	_inc_command_vel[1] = (int32_t)round(qdot(1)/(2.0*M_PI)*(66.0*1600.0));

	HandMotion_p = HandMotion;
}

void KistHand::ForKinematics_Thumb(digit *pThumb)
{
    MatrixXd tT01(4, 4), tT12(4, 4), tT23(4, 4), tT34(4, 4), tT45(4, 4), tT56(4, 4), tT67(4, 4), tT78(4, 4), tT89(4, 4);
    MatrixXd tT910(4, 4), tT1011(4, 4), tT1112(4, 4), tT1213(4, 4), tT013(4, 4);


    tT01 << cos(qt), sin(qt), 0, 0, -sin(qt), cos(qt), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT12 << 1, 0, 0, Lt[0], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT23 << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT34 << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    tT45 << cos(pThumb->q[0]), -sin(pThumb->q[0]), 0, 0, sin(pThumb->q[0]), cos(pThumb->q[0]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT56 << 1, 0, 0, Lt[1], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT67 << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1;
    tT78 << cos(pThumb->q[1]), -sin(pThumb->q[1]), 0, 0, sin(pThumb->q[1]), cos(pThumb->q[1]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT89 << 1, 0, 0, Lt[2], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT910 << cos(pThumb->q[2]), -sin(pThumb->q[2]), 0, 0, sin(pThumb->q[2]), cos(pThumb->q[2]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT1011 << 1, 0, 0, Lt[3], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT1112 << cos(pThumb->q[3]), -sin(pThumb->q[3]), 0, 0, sin(pThumb->q[3]), cos(pThumb->q[3]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    tT1213 << 1, 0, 0, Lt[4], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    tT013 = tT01 * tT12 * tT23 * tT34 * tT45 * tT56 * tT67 * tT78 * tT89 * tT910 * tT1011 * tT1112 * tT1213;

    for (int i = 0; i <= 2; i++)
    	pThumb->p[i] = tT013(i, 3);
}

void KistHand::ForKinematics_Fingers(digit *pFinger, int Pos )
{
    float Lbs;
    MatrixXd fT01(4, 4), fT12(4, 4), fT23(4, 4), fT34(4, 4), fT45(4, 4), fT56(4, 4), fT06(4, 4);

    if (Pos == 1)
    	Lbs = Lb;
    else
    	Lbs = -Lb;

    fT01 << 0, -1, 0, -Lbs, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    fT12 << cos(pFinger->q[0]), -sin(pFinger->q[0]), 0, Lp, sin(pFinger->q[0]), cos(pFinger->q[0]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    fT23 << cos(pFinger->q[1]), -sin(pFinger->q[1]), 0, L[0], 0, 0, -1, 0, sin(pFinger->q[1]), cos(pFinger->q[1]), 0, 0, 0, 0, 0, 1;
    fT34 << cos(pFinger->q[2]), -sin(pFinger->q[2]), 0, L[1], sin(pFinger->q[2]), cos(pFinger->q[2]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    fT45 << cos(pFinger->q[3]), -sin(pFinger->q[3]), 0, L[2], sin(pFinger->q[3]), cos(pFinger->q[3]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    fT56 << 1, 0, 0, L[3], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    fT06 = fT01 * fT12 * fT23 * fT34 * fT45 * fT56;
    for (unsigned int i = 0; i < 3; i++)
    {
        pFinger->p[i] = fT06(i, 3);
    }
}

void KistHand::InvKinematics_Thumb(digit *pThumb )
{
    float pdt[2];
    float C1, S1, K1, K2;

    // Solve theta 0
    pdt[0] = pThumb->p[0] * sin(qt) + cos(qt) * pThumb->p[1];
    pdt[1] = pThumb->p[2];
    pThumb->q[0] = atan2(pdt[1], pdt[0]);

    // Solve theta 2
    pdt[0] = cos(pThumb->q[0]) * sin(qt) * pThumb->p[0] + cos(pThumb->q[0]) * cos(qt) * pThumb->p[1] + sin(pThumb->q[0]) * pThumb->p[2] - Lt[1] - Lt[4] * cos(fth);
    pdt[1] = -cos(qt) * pThumb->p[0] + sin(qt) * pThumb->p[1] + Lt[0] - Lt[4] * sin(fth);
    C1 = (pow((pdt[0]), 2) + pow((pdt[1]), 2) - pow(Lt[2], 2) - pow(Lt[3], 2)) / (2 * Lt[2] * Lt[3]);
    S1 = sqrt(1 - pow(C1, 2));
    pThumb->q[2] = atan2(S1, C1);

    // Solve theta 1
    K1 = Lt[2] + Lt[3] * C1;
    K2 = Lt[3] * S1;

    pThumb->q[1] = atan2(pdt[1], pdt[0]) - atan2(K2, K1);
    pThumb->q[3] = (fth - pThumb->q[1] - pThumb->q[2]);
}

void KistHand::InvKinematics_Fingers(digit *pFinger, int Pos )
{
    float pk, pf, df, ddf, dkf, Lbs;
    float hkf[4], kf[3], pdf[2];

    //Solve theta 1 of each finger
    if (Pos == 1) Lbs = Lb;
    else        Lbs = -Lb;

    pFinger->q[0] = atan2((Lbs + pFinger->p[0]), -(pFinger->p[1] - Lp));
    if (pFinger->q[0] < -M_PI / 2) {
            pFinger->q[0] = atan2(-(Lbs + pFinger->p[0]), (pFinger->p[1] - Lp));
    }

    //Solve theta 3,4
    pdf[0] = -sin(pFinger->q[0]) * pFinger->p[0] + cos(pFinger->q[0]) * pFinger->p[1] - L[0] - Lp * cos(pFinger->q[0]) - Lb * sin(pFinger->q[0]);
    hkf[0] = pow(pdf[0], 2) + pow(pFinger->p[2], 2) - pow(L[1], 2) - pow(L[2], 2) - pow(L[3], 2);
    hkf[1] = 2 * L[1] * L[2];
    hkf[2] = 2 * L[2] * L[3];
    hkf[3] = 2 * L[1] * L[3];
    kf[0] = hkf[1] / 24 + pow(k, 4) * (hkf[2] / 24) + pow(1 + k, 4) * (hkf[3] / 24);
    kf[1] = -hkf[1] / 2 - pow(k, 2) * (hkf[2] / 2) - pow(1 + k, 2) * (hkf[3] / 2);
    kf[2] = hkf[1] + hkf[2] + hkf[3] - hkf[0];
    pf = pow(pdf[0],2) + pow(pFinger->p[2],2) - pow(L[1],2) - pow(L[2],2) - pow(L[3],2);
    pFinger->q[2] = sqrt((-kf[1] - sqrt(pow(kf[1], 2) - 4 * kf[0] * kf[2])) / (2 * kf[0]));
    pFinger->q[3] = k * pFinger->q[2];

    for (int i = 1; i <= 5; i++) // interpolation to reduce error
    {
        dkf = -2 * L[1] * L[2] * sin(pFinger->q[2]) - 2 * L[1] * L[3] * (k + 1) * sin(pFinger->q[2] + pFinger->q[3]) - 2 * L[2] * L[3] * k * sin(pFinger->q[3]);
        pk = 2 * L[1] * L[2] * cos(pFinger->q[2]) + 2 * L[1] * L[3] * cos(pFinger->q[2] + pFinger->q[3]) + 2 * L[2] * L[3] * cos(pFinger->q[3]);
        pFinger->q[2] = pFinger->q[2] + (pf - pk) / (dkf);
        pFinger->q[3] = k * pFinger->q[2];
    }

    //Solve theta 2
    df = (L[2] * sin(pFinger->q[2]) + L[3] * sin(pFinger->q[2] + pFinger->q[3]));
    ddf = L[1] + L[2] * cos(pFinger->q[2]) + L[3] * cos(pFinger->q[2] + pFinger->q[3]);
    pFinger->q[1] = atan2((pFinger->p[2] * ddf - pdf[0] * df), (pdf[0] * ddf + pFinger->p[2] * df));
}

} /* namespace hyuCtrl */
