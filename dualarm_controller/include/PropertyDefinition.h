#pragma once
/**
 * @file PropertyDefinition.h
 * @date 2019-06-10
 * @author Junho Park
 * @version 1.0.0
 */

#define ROBOT_DOF 14 /**< Number of Manipulator joint*/
//#define _WITH_HAND_

/**
 * @brief Kinematics Infomation
 */
typedef struct{
	double w_x; 	/**< Twist x-axis*/
	double w_y;		/**< Twist y-axis*/
	double w_z;		/**< Twist z-axis*/
	double q_x;		/**< Twist x-axis*/
	double q_y;		/**< Twist y-axis*/
	double q_z;		/**< Twist z-axis*/
	double l_x;		/**< Initial Configuration x-axis*/
	double l_y;		/**< Initial Configuration y-axis*/
	double l_z;		/**< Initial Configuration z-axis*/
}robot_kinematic_info;

/**
 * @brief Dynamic Information
 */
typedef struct{
	double mass_kg; 	/**< mass of each link in kg*/
	double Ixx_kgm2; 	/**< xx direction inertia in kgm^2*/
	double Iyy_kgm2; 	/**< xy direction inertia in kgm^2*/
	double Izz_kgm2; 	/**< xz direction inertia in kgm^2*/
	double Ixy_kgm2; 	/**< yy direction inertia in kgm^2*/
	double Iyz_kgm2; 	/**< yz direction inertia in kgm^2*/
	double Izx_kgm2; 	/**< zz direction inertia in kgm^2*/
	double CoM_x;		/**< x direction CoM in m*/
	double CoM_y;		/**< y direction CoM in m*/
	double CoM_z;		/**< z direction CoM in m*/
}robot_dynamic_info;

/**
 * @brief motor information
 */
typedef struct{
	int 	motor_harmonic; 		/**< gear ratio*/
	int 	enc_size; 				/**< resolution of encoder  */
	double 	max_current_A;			/**< maximum continuous current of motor */
	double 	torque_const_Nm_A; 		/**< Torque constant of motor in Nm A*/
	int 	Offset;					/**< Absolute Encoder Zero-position Offset*/
}robot_motor_info;

typedef struct{
	double VelThreshold;
	double FrictionTorque;
}FrictionMap;

typedef struct{
	double a;
	double b;
	double c;
	double d;
	double e;
	double f;
}FrictionTanh;

/**
 * @brief Predefined kinematic object
 */
extern robot_kinematic_info serial_Kinematic_info[];

/**
 * @brief Predefined dynamic object
 */
extern robot_dynamic_info serial_Dynamic_info[];

/**
 * @brief Predefined system object
 */
extern robot_motor_info serial_Motor_info[];

extern FrictionMap frictionmap[];

extern FrictionTanh frictiontanh[];
