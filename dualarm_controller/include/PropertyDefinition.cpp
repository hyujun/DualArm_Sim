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

#define HARMONIC_100 101
#define ABS_ENC_19 524288

#define MAX_CURRENT_TYPE1 4.3
#define MAX_CURRENT_TYPE2 2.46
#define MAX_CURRENT_TYPE3 2.78
#define MAX_CURRENT_TYPE4 3.79

#define TORQUE_CONST_TYPE1 0.341 // Nm/A Arm 1,2,3
#define TORQUE_CONST_TYPE2 0.426 // Nm/A Arm 4
#define TORQUE_CONST_TYPE3 0.213 // Nm/A Arm 5,6
#define TORQUE_CONST_TYPE4 0.683 // Nm/A Waist 1,2


// {w_x, w_y, w_z, q_x, q_y, q_z, l_x, l_y, l_z}
robot_kinematic_info serial_Kinematic_info[] = {
		{0, 0, 1,
				0.0, 0.0, 352.0e-3,
				0.0, 0.0, 500.0e-3},			// 1
		{0, -1, 0,
				0.0, 0.0, 500.0e-3,
				0.0, -125.0e-3, 717.0e-3},		// 2
//Right hand
		{0, 0, -1,
				0.0, -125.0e-3, 717.0e-3,
				0.0, -215.0e-3, 800.0e-3},		// 3
		{0, 1, 0,
				0.0, -215.0e-3, 800.0e-3,
				0.0, -325.0e-3, 800.0e-3},		// 4
		{1, 0, 0,
				0.0, -325.0e-3, 800.0e-3,
				0.0, -325.0e-3, 558.0e-3},		// 5
		{0, 0, 1,
				0.0, -325.0e-3, 558.0e-3,
				-24.5e-3, -325.0e-3, 400.0e-3},// 6
		{0, 1, 0,
				-24.5e-3, -325.0e-3, 400.0e-3,
				0.0, -325.0e-3, 245.0e-3},		// 7
#if defined(_WITH_HAND_)
		{0, 0, 1,
				0.0, -325.0e-3, 245.0e-3,
				0.0, -325.0e-3, 235.0e-3},		// 8
#else
		{0, 0, 1,
				0.0, -325.0e-3, 245.0e-3,
				0.0, -325.0e-3, 235.0e-3},		// 8
#endif
//Left hand
		{0, 0, -1,
				0.0, 125.0e-3, 717.0e-3,
				0.0, 215.0e-3, 800.0e-3},		// 9
		{0, -1, 0,
				0.0, 215.0e-3, 800.0e-3,
				0.0, 325.0e-3, 800.0e-3},		// 10
		{1, 0, 0,
				0.0, 325.0e-3, 800.0e-3,
				0.0, 325.0e-3 ,558.0e-3},		// 11
		{0, 0, 1,
				0.0, 325.0e-3, 558.0e-3,
				-24.5e-3, 325.0e-3, 400.0e-3},	// 12
		{0, -1, 0,
				-24.5e-3, 325.0e-3, 400.0e-3,
				0.0, 325.0e-3, 245.0e-3},		// 13
#if defined(_WITH_HAND_)
		{0, 0, 1,
				0.0, 325.0e-3, 245.0e-3,
				0.0, 299.5e-3, 5.21e-3},		// 14
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
		{10.91,
				0.0463, 0.0367, 0.0271, 0.0, 0.0026849, 0.0,
				0.0354e-3, 4.4067e-3, 463.0776e-3}, 		// 1
		{12.2662,
				0.1408, 0.05, 0.108, 0.0, 0.003643, 0.0007,
				1.0139e-3, 3.6731e-3, 643.6171e-3},			// 2
//Right
		{3.35,
				0.0057, 0.0055, 0.0044, 0.0, -0.0005, 0.0,
				0.2952e-3, -159.1703e-3, 793.4610e-3},		// 3
		{3.51,
				0.0088, 0.0049, 0.0089, -0.000115, 0.0, 0.0,
				11.2502e-3, -310.0175e-3, 800.1092e-3},		// 4
		{2.59,
				0.0103, 0.0112, 0.003, 0.0, 0.0007, 0.0007,
				1.5675e-3, -325.3615e-3, 636.6232e-3},		// 5
		{2.49,
				0.0077, 0.0068, 0.0026, 0.0, 0.0, 0.0007,
				21.6024e-3, -314.6943e-3, 423.8501e-3},		// 6
		{1.9,
				0.0039, 0.0034, 0.0018, 0.0, 0.0002, 0.0002,
				1.3060e-3, -324.0871e-3, 292.6109e-3},		// 7
#if defined(_WITH_HAND_)
		{2.1946,
				19608890.26e-9, 22095290.66e-9, 4276220.1e-9, 165563.4e-9, -1869503.6e-9, -1055812.83e-9,
				7.04e-3, -311.32e-3, 76.65e-3},			// 8
#else
		{0.0646,
				0.0005, 0.0005, 0.0005, 0.0, 0.0, 0.0,
				0.0e-3, -322.2826e-3, 240.1051e-3},			// 8
#endif
//Left
		{3.35,
				0.0057, 0.0055, 0.0044, 0.0, -0.0005, 0.0,
				0.2952e-3, 159.1703e-3, 793.4610e-3},		// 9
		{3.51,
				0.0088, 0.0049, 0.0089, -0.000115, 0.0, 0.0,
				11.2502e-3, 310.0175e-3, 800.1092e-3},		// 10
		{2.59,
				0.0103, 0.0112, 0.003, 0.0, 0.0007, 0.0007,
				1.5675e-3, 325.3615e-3, 636.6232e-3},		// 11
		{2.49,
				0.0077, 0.0068, 0.0026, 0.0, 0.0, -0.0007,
				21.6024e-3, 314.6943e-3, 423.8501e-3},		// 12
		{1.9,
				0.0039, 0.0034, 0.0018, 0.0, 0.0002, 0.0002,
				1.3060e-3, 324.0871e-3, 292.6109e-3},		// 13
#if defined(_WITH_HAND_)
		{2.1946,
				64638829.47e-9, 66659017.77e-9, 4566899.4e-9, -182585.55e-9, 2023270.8e-9, -1132418.18e-9,
				5.85e-3, 313.26e-3, 101.72e-3},			// 8
#else
		{0.0646,
				0.0005, 0.0005, 0.0005, 0.0, 0.0, 0.0,
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

		{1.287, 1.641, 165.9, 7.541, 19.43, 0.1302}, 		//9
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7}, 		//10
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7}, 		//11
		{325.2, 22, 22.72, 4.02, 28.35, 0.07359}, 			//12
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//13
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//14
};
