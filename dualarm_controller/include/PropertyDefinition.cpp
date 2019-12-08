#include "PropertyDefinition.h"

#define ENC_OFFSET_J1 -51476200
#define ENC_OFFSET_J2 16350230
#define ENC_OFFSET_J3 104515000
#define ENC_OFFSET_J4 504872667
#define ENC_OFFSET_J5 14598364
#define ENC_OFFSET_J6 10983523
#define ENC_OFFSET_J7 -6860733
#define ENC_OFFSET_J8 -59420800
#define ENC_OFFSET_J9 185495795
#define ENC_OFFSET_J10 11226000
#define ENC_OFFSET_J11 -11773634
#define ENC_OFFSET_J12 12912330
#define ENC_OFFSET_J13 19522733
#define ENC_OFFSET_J14 37893089

#define ENC_OFFSET_WRIST 50578817

#define HARMONIC_100 101
#define ABS_ENC_19 524288

#define MAX_CURRENT_TYPE1 4.3
#define MAX_CURRENT_TYPE2 2.46
#define MAX_CURRENT_TYPE3 2.78
#define MAX_CURRENT_TYPE4 3.79
#define MAX_CURRENT_WRIST 2.31

#define TORQUE_CONST_TYPE1 0.341 // Nm/A Arm 1,2,3
#define TORQUE_CONST_TYPE2 0.426 // Nm/A Arm 4
#define TORQUE_CONST_TYPE3 0.213 // Nm/A Arm 5,6
#define TORQUE_CONST_TYPE4 0.683 // Nm/A Waist 1,2
#define TORQUE_CONST_WRIST 0.091 // Nm/A Wrist


// {w_x, w_y, w_z, q_x, q_y, q_z, l_x, l_y, l_z}
robot_kinematic_info serial_Kinematic_info[] = {
		{0, 0, 1,
				0.0, 0.0, 352.0e-3+300e-3,
				0.0, 0.0, 500.0e-3+300e-3},			// 1
		{0, -1, 0,
				0.0, 0.0, 500.0e-3+300e-3,
				0.0, 0.0e-3, 800.0e-3+300e-3},		// 2
//Right hand
		{0, 0, -1,
				0.0, -125.0e-3, 717.0e-3+300e-3,
				0.0, -215.0e-3, 800.0e-3+300e-3},		// 3
		{0, 1, 0,
				0.0, -215.0e-3, 800.0e-3+300e-3,
				0.0, -325.0e-3, 800.0e-3+300e-3},		// 4
		{1, 0, 0,
				0.0, -325.0e-3, 800.0e-3+300e-3,
				0.0, -325.0e-3, 558.0e-3+300e-3},		// 5
		{0, 0, 1,
				0.0, -325.0e-3, 558.0e-3+300e-3,
				-24.5e-3, -325.0e-3, 400.0e-3+300e-3},     // 6
		{0, 1, 0,
				-24.5e-3, -325.0e-3, 400.0e-3+300e-3,
				0.0, -325.0e-3, 245.0e-3+300e-3},		// 7
#if defined(_WITH_HAND_)
		{0, 0, 1,
				0.0, -325.0e-3, 245.0e-3+300e-3,
				0.0, -325.0e-3, 29.0e-3+300e-3},		// 8

		{1, 0, 0,
				0.0, -325.0e-3, 29.0e-3+300e-3,
				0.0, -299.52e-3, -120.8e-3+300e-3},		// Wrist
#else
		{0, 0, 1,
				0.0, -325.0e-3, 245.0e-3,
				0.0, -325.0e-3, 235.0e-3},		// 8

#endif
//Left hand
		{0, 0, -1,
				0.0, 125.0e-3, 717.0e-3+300e-3,
				0.0, 215.0e-3, 800.0e-3+300e-3},		// 9
		{0, -1, 0,
				0.0, 215.0e-3, 800.0e-3+300e-3,
				0.0, 325.0e-3, 800.0e-3+300e-3},		// 10
		{1, 0, 0,
				0.0, 325.0e-3, 800.0e-3+300e-3,
				0.0, 325.0e-3 ,558.0e-3+300e-3},		// 11
		{0, 0, 1,
				0.0, 325.0e-3, 558.0e-3+300e-3,
				-24.5e-3, 325.0e-3, 400.0e-3+300e-3},	// 12
		{0, -1, 0,
				-24.5e-3, 325.0e-3, 400.0e-3+300e-3,
				0.0, 325.0e-3, 245.0e-3+300e-3},		// 13
#if defined(_WITH_HAND_)
		{0, 0, 1,
				0.0, 325.0e-3, 245.0e-3+300e-3,
				0.0, 299.5e-3, 2.21e-3+300e-3},		// 14
#else
		{0, 0, 1,
				0.0, 325.0e-3, 245.0e-3,
				0.0, 325.0e-3, 235.0e-3},		// 14
#endif
};

//{MASS,
// J_Ixx, J_Iyy, J_Izz, J_Ixy, J_Iyz, J_Izx,
// CoM_x, CoM_y, CoM_z},
robot_dynamic_info serial_Dynamic_info[] = {
//body
		{11.91,
				83854563.43e-9, 70969371.34e-9, 53062013.96e-9, -135841.54e-9, -15432807.98e-9, -625787.35e-9,
				-0.1e-3, -3.05e-3, 449.99e-3+300e-3}, 		// 1
		{9.901,
				0.1408, 0.05, 0.108, 0.0, 0.003643, 0.0007,
				1.0139e-3, 3.6731e-3, 643.6171e-3+300e-3},			// 2
//Right
		{3.35,
				9155194.92e-9, 7656654.39e-9, 6023745.1e-9, 163898.7e-9, -981692.47e-9, -52182.13e-9,
				-0.7e-3, -142.89e-3, 781.38e-3+300e-3},		// 3
		{3.51,
				11833182.94e-9, 5963801.91e-9, 12674379.1e-9, 245028.38e-9, -81204.78e-9, -121068.48e-9,
				-0.21e-3, -293.04e-3, 800.77e-3+300e-3},		// 4
		{2.59,
				14349158.07e-9, 15957517.14e-9, 3811118.16e-9, 0.0, -1126404.95e-9, 0.0,
				-2.54e-3, -325.18e-3, 649.61e-3+300e-3},		// 5
		{2.49,
				12305788.09e-9, 10949889.37e-9, 3675298.10e-9, 0.0, -322614.65e-9, 1326295.35e-9,
				-19.09e-3, -322.41e-3, 449.31e-3+300e-3},		// 6
		{1.8,
				4927170.75e-9, 4076854.09e-9, 2250292.68e-9, 0.0, 298769.64e-9, -353301.92e-9,
				-2.33e-3, -322.87e-3, 303.51e-3+300e-3},		// 7
#if defined(_WITH_HAND_)
		{2.676,
				15715348.78e-9, 16761167.93e-9, 3543832.43e-9, 0.0, 0.0, -347842.21e-9,
				1.73e-3, -325.14e-3, 107.78e-3+300e-3},			// 8

		{1.29,
				7534883.39e-9, 9485578.12e-9, 25880996.64e-9, 0.0, -659219.99e-9, 0.0,
				13.82e-3, -307.13e-3, -93.77e-3+300e-3},			// Wrist
#else
		{0.0646,
				0.00005, 0.00005, 0.00005, 0.0, 0.0, 0.0,
				0.0e-3, -322.2826e-3, 240.1051e-3},			// 8
#endif
//Left
		{3.35,
				0.0057, 0.0055, 0.0044, 0.0, -0.0005, 0.0,
				-0.87e-3, 142.89e-3, 781.36e-3+300e-3},		// 9
		{3.51,
				11625561.71e-9, 6340663.94e-9, 13098282.6e-9, -73542.31e-9, 817088484.13e-9, 2404534.61e-9,
				0.89e-3, 290.77e-3, 800.53e-3+300e-3},		// 10
		{2.59,
				14349157.75e-9, 15959292.97e-9, 3812893.68e-9, 0.0, 0.0, -1118640.19e-9,
				-2.38e-3, 325.16e-3, 649.61e-3+300e-3},		// 11
		{2.49,
				12366056.79e-9, 11004463.34e-9, 3670383.10e-9, 0.0, 0.0, 1314062.31e-9,
				-18.91e-3, 322.25e-3, 449.78e-3+300e-3},		// 12
		{1.8,
				4932126.5e-9, 4076854.09e-9, 2255248.43e-9, 80829.44e-9, -355897.62e-9, -353301.92e-9,
				-2.31e-3, 323.63e-3, 303.51e-3+300e-3},		// 13
#if defined(_WITH_HAND_)
		{2.1946,
				64638829.47e-9, 66659017.77e-9, 4566899.4e-9, -182585.55e-9, 2023270.8e-9, -1132418.18e-9,
				8.24e-3, 309.28e-3, 54.84e-3+300e-3},			// 8
#else
		{0.0646,
				0.00005, 0.00005, 0.00005, 0.0, 0.0, 0.0,
				0.0e-3, 322.2826e-3, 240.1051e-3},			// 8
#endif
};

// {HarmonicRatio, EncoderResolution, MaximumContinuousCurrent, TorqueConstant, AbsolutePositionOffset}
robot_motor_info serial_Motor_info[] = {
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE4, TORQUE_CONST_TYPE4, ENC_OFFSET_J1},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE4, TORQUE_CONST_TYPE4, ENC_OFFSET_J2},

		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J3},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J4},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J5},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE2, TORQUE_CONST_TYPE2, ENC_OFFSET_J6},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J7},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J8},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_WRIST, TORQUE_CONST_WRIST, ENC_OFFSET_WRIST},

		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J9},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J10},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J11},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE2, TORQUE_CONST_TYPE2, ENC_OFFSET_J12},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J13},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J14},
};

FrictionMap frictionmap[] ={
		{0.05, 18.5},	//1
		{0.05, 10.5},	//2

		{0.05, 6.8},	//3
		{0.05, 7.0},	//4
		{0.05, 6.5},	//5
		{0.05, 4.0},	//6
		{0.03, 3.3},	//7
		{0.03, 2.5},	//8

		{0.05, 7.1},	//9
		{0.05, 7.0},	//10
		{0.05, 6.5},	//11
		{0.05, 4.0},	//12
		{0.03, 3.3},	//13
		{0.03, 2.5},	//14
};

FrictionTanh frictiontanh[] = {
		{215, 24.83, 22.54, 20.5, 11.81, 0.715}, 			//1
		{215, 24.83, 22.54, 34.7, 11.81, 0.715},	 		//2

		{41.87, 573.8, 592.2, 6.811, 12.96, 0.138}, 		//3
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7},		//4
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7},		//5
		{151.3, 16.42, 17.26, 4.56, 14.47, 0.05998}, 		//6
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//7
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//8
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//Wrist

		{1.287, 1.641, 165.9, 7.541, 19.43, 0.1302}, 		//9
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7}, 		//10
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7}, 		//11
		{325.2, 22, 22.72, 4.02, 28.35, 0.07359}, 			//12
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//13
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//14
};
