#include "constant_def.h"
#include "kinematic.h"
#include "mbed.h"



robot_kinematic::robot_kinematic(float alpha) { //not EWMA alpha
    m_alpha = alpha;
    return; 
}


void robot_kinematic::cal(int Servo_Raw) {
    // Servo_Bearing_Radian = Setpoint (Pi/2) - process value (Servo_Pos in radians)
    m_Servo_Bearing_Radian = (M_PI/2) - (Servo_Raw - Servo_Lower_Limit)*Servo_Raw_To_Radian;
    
    if (cos(2*m_Servo_Bearing_Radian) != 1.0) { //check if Servo_Bearing inside working space range
        m_Curve_Radius =  sqrt((0.0101+(0.0099*cos(m_Servo_Bearing_Radian)))/(1-cos(2*m_Servo_Bearing_Radian))); //Radius in  meter
        m_Speed_Factor = (1/(1+exp(-m_alpha*m_Curve_Radius)))-0.5; // sigmoid unit conversion

        m_Velocity = m_Speed_Factor*Robot_Max_Power*Robot_Velcity_Per_Percent_Power; // unit conversion into m/s

        if (Servo_Raw - Servo_Mid > 0) {
            m_Angular_Velocity = m_Velocity/m_Curve_Radius; //angular velocity in positive value (robot turn )
        }
        else {
            m_Angular_Velocity = -m_Velocity/m_Curve_Radius; //angular velocity in negative value (robot turn )
        }
        
    }

    m_Left_M_Speed = Angular_Velocity_To_Motor_Power*(m_Velocity-(0.5*m_Angular_Velocity*Wheel_width))/Wheel_Radius;
    m_Right_M_Speed = Angular_Velocity_To_Motor_Power*(m_Velocity+(0.5*m_Angular_Velocity*Wheel_width))/Wheel_Radius;

}



float robot_kinematic::get_Servo_Bearing_Radian() {
    return m_Servo_Bearing_Radian;
}

float robot_kinematic::get_Angular_Velocity() {
    return m_Angular_Velocity;
}

float robot_kinematic::get_Radius() {
    return m_Curve_Radius; //radius = 0 indicate that there is no curvature
}

float robot_kinematic::get_Speed_factor() {
    return m_Speed_Factor;
}

float robot_kinematic::get_velocity(){
    return m_Velocity;
}

float robot_kinematic::Left_M_Speed() {
    return m_Left_M_Speed;
}

float robot_kinematic::Right_M_Speed() {
    return m_Right_M_Speed;
}