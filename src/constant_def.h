#ifndef CONSTANT_DEF_H
#define CONSTANT_DEF_H

// constant define
#define M_PI 3.14159265358979323846

// robot physical dimension in meter and radian
#define Wheel_width 0.13
// Robot_Arm_Lenght 0.11
// Robot_Body_Lenght 0.08
#define Wheel_Radius 0.01625 //m
#define Robot_Max_Power 0.18
#define Robot_Velcity_Per_Percent_Power 0.06 // 1.2 m/s @ 20 percent power (0.05 per percent power)
#define Robot_Angular_Velocity_Per_Percent_Power 3.69 // 73.8 radian/s @20 percent power (3.69 per percent power)

#define Angular_Velocity_To_Motor_Power (1/3.69)

#define Num_of_Sensor 6


//cut out sensor #1 and #8
#define Sen_1_Pin PA_7 // (A6)
#define Sen_2_Pin PA_6 // (A5)
#define Sen_3_Pin PA_5 // (A4)
#define Sen_4_Pin PA_4 // (A3)
#define Sen_5_Pin PA_3 // (A2)
#define Sen_6_Pin PA_1 // (A1)

#define Robot_LED4 PB_4

#define SW_1_Pin PB_7

#define Servo_Poten_Pin PB_0

#define Servo_In_1_Pin PB_1
#define Servo_In_2_Pin PB_6

#define R_Motor_PWM_Pin D9
#define R_Motor_Sig_Pin D8

#define L_Motor_PWM_Pin D10
#define L_Motor_Sig_Pin D11

#define Servo_Lower_Limit 6400
#define Servo_Upper_Limit 60000
#define Servo_Mid 33200

#define Servo_Raw_To_Radian M_PI/(Servo_Upper_Limit - Servo_Lower_Limit)

#define Center_Line_Val 35
#define Sen_Mirroring 1


#endif



