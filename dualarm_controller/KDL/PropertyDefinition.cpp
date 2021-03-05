#include "PropertyDefinition.h"

#define ENC_OFFSET_J1 6434393
#define ENC_OFFSET_J2 16421600
#define ENC_OFFSET_J3 104454976
#define ENC_OFFSET_J4 506083805
#define ENC_OFFSET_J5 14399894
#define ENC_OFFSET_J6 10801285
#define ENC_OFFSET_J7 2921664
#define ENC_OFFSET_J8 -49482190

#define ENC_OFFSET_J9 185787856
#define ENC_OFFSET_J10 11739287
#define ENC_OFFSET_J11 -14866250
#define ENC_OFFSET_J12 12812683
#define ENC_OFFSET_J13 19996194
#define ENC_OFFSET_J14 19796963

#define ENC_OFFSET_WRIST1 -56659968
#define ENC_OFFSET_WRIST2 100815317

#define ENC_OFFSET_WRIST 44619175

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
// -----------------------
//body(waist)
// -----------------------
		{0, 0, -1,
				0.0, 0.0, 303.1e-3,
				0.0, 0.0, 500.0e-3},			// 1
		{0, -1, 0,
				0.0, 0.0, 500.0e-3,
				0.0, -125.0e-3, 685.65e-3},		// 2
//-----------------------
//Right-side arm
// -----------------------
		{0, 0, -1,
				0.0, -125.0e-3, 685.65e-3,
				0.0, -177.65e-3, 800.0e-3},		// 3
		{0, 1, 0,
				0.0, -177.65e-3, 800.0e-3,
				0.0, -325.0e-3, 800.0e-3},		// 4
		{1, 0, 0,
				0.0, -325.0e-3, 800.0e-3,
				0.0, -325.0e-3, 585.75e-3},		// 5
		{0, 0, 1,
				0.0, -325.0e-3, 585.75e-3,
				-24.5e-3, -325.0e-3, 400.0e-3},     // 6
		{0, 1, 0,
				-24.5e-3, -325.0e-3, 400.0e-3,
				0.0, -325.0e-3, 260.0e-3},		// 7
		{0, 0, 1,
				0.0, -325.0e-3, 260.0e-3,
                0.0, -325.0e-3, 169.0e-3},		// 8
		{1, 0, 0,
				0.0, -325.0e-3, 169.0e-3,
                0.0, -325.0e-3, 122.19e-3},		// Wrist
//-----------------------
//Left-side arm
//-----------------------
		{0, 0, -1,
				0.0, 125.0e-3, 685.65e-3,
                0.0, 177.65e-3, 800.0e-3},		// 9
		{0, -1, 0,
				0.0, 177.65e-3, 800.0e-3,
				0.0, 325.0e-3, 800.0e-3},		// 10
		{1, 0, 0,
				0.0, 325.0e-3, 800.0e-3,
                0.0, 325.0e-3, 585.75e-3},		// 11
		{0, 0, 1,
				0.0, 325.0e-3, 585.75e-3,
                -24.5e-3, 325.0e-3, 400.0e-3},	// 12
		{0, -1, 0,
				-24.5e-3, 325.0e-3, 400.0e-3,
            0.0, 325.0e-3, 260.0e-3},		// 13
		{0, 0, 1,
				0.0, 325.0e-3, 260.0e-3,
                0.0, 325.0e-3, 179.0e-3},	// 14
		{1, 0, 0,
				0.0, 325.0e-3, 179.0e-3,
				0.0, 325.0e-3, 122.19e-3},		// wrist2
};

//{MASS,
// J_Ixx, J_Ixy, J_Ixz, J_Iyy, J_Iyz, J_Izz,
// CoM_x, CoM_y, CoM_z},
robot_dynamic_info serial_Dynamic_info[] = {
// -----------------------
//body(waist)
// -----------------------
		{11.91,
				0.094, -0.0002, -0.0001, 0.0712, 0.0005, 0.0565,
				-0.0001, -0.0016, 0.4551}, 		// waist1, Joint1
		{10.901,
				0.1614, -0.0002, -0.0001, 0.0712, 0.0005, 0.0565,
				0.0054, 0.0042, 0.619},			// waist2, Joint2
//-----------------------
//Right-side arm
// -----------------------
		{3.35,
				0.094, 0.0002, -0.0001, 0.0077, -0.001, 0.006,
				-0.0007, -0.1429, 0.7812},		// right-arm1, Joint3
		{3.51,
				0.0118, 0.0002, -0.0001, 0.006, -0.0001, 0.0127,
				-0.0002, -0.293, 0.8008},		// right-arm2, Joint4
		{2.59,
				0.0143, 0.0, -0.0011, 0.016, 0.0, 0.0038,
				-0.0025, -0.3252, 0.6496},	    // right-arm3, Joint5
		{2.49,
				0.0123, -0.0001, 0.0013, 0.0109, -0.0003, 0.0037,
				-0.0191, -0.3224, 0.4493},		// right-arm4, Joint6
		{1.8,
				0.0049, -0.0001, -0.0004, 0.0041, 0.0003, 0.0023,
				-0.0023, -0.3229, 0.3035},		// right-arm5, Joint7
		{1.73,
				0.002, 0.0, -0.0001, 0.0029, 0.0, 0.0021,
				0.0039, -0.3252, 0.1805},		// right-arm6, Joint8
		{1.08,
				0.0071, 0.0003, 0.0002, 0.0082, -0.0003, 0.0022,
				0.0098, -0.3164, 0.043},			// right-arm7, Joint9, Wrist1, Allegro hand
//-----------------------
//Left-side arm
//-----------------------
		{3.35,
				0.0092, -0.0001, 0.0, 0.0077, 0.001, 0.006,
				-0.0009, 0.1429, 0.7814},	    // left-arm1, Joint10
		{3.51,
				0.0116, -0.0001, -0.0001, 0.0063, 0.0001, 0.0131,
				0.0009, 0.2908, 0.8005},		    // left-arm2, Joint11
		{2.59,
				0.0143, 0.0, -0.0011, 0.016, 0.0, 0.0038,
				-0.0024, 0.3252, 0.6496},		// left-arm3, Joint12
		{2.49,
				0.0124, 0.0001, 0.0013, 0.011, 0.0003, 0.0037,
				-0.0189, 0.3223, 0.4498},	    // left-arm4, Joint13
		{1.8,
				0.0049, 0.0001, -0.0004, 0.0041, -0.0004, 0.0023,
				-0.0023, 0.3248, 0.3035},		// left-arm5, Joint14
		{1.73,
				0.0002, 0.0, -0.0001, 0.0028, 0.0, 0.0021,
				0.0039, 0.3248, 0.18},	// left-arm6, Joint15
		{1.147,
				0.0057, -0.0002, -0.0007, 0.0072, 0.0006, 0.0027,
				0.0188, 0.3093, 0.0588},			// left-arm7, Joint16, Wrist2, None
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
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_WRIST, TORQUE_CONST_WRIST, ENC_OFFSET_WRIST1},

		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J9},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J10},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE1, TORQUE_CONST_TYPE1, ENC_OFFSET_J11},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE2, TORQUE_CONST_TYPE2, ENC_OFFSET_J12},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J13},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_TYPE3, TORQUE_CONST_TYPE3, ENC_OFFSET_J14},
		{HARMONIC_100, ABS_ENC_19, MAX_CURRENT_WRIST, TORQUE_CONST_WRIST, ENC_OFFSET_WRIST2},
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
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//wrist1

		{1.287, 1.641, 165.9, 7.541, 19.43, 0.1302}, 		//9
		{19.99, 35.09, 64.18, 17.6, 36.34, 2.398e-7}, 		//10
		{2.069, 59.79, 95.02, 18.5, 3.299, 2.49e-7}, 		//11
		{325.2, 22, 22.72, 4.02, 28.35, 0.07359}, 			//12
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//13
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//14
		{2.444, 3.931, 159.1, 3.963, 77.64, 0.0543}, 		//wrist2
};
