#ifndef KINEMATIC_H
#define KINEMATIC_H


class robot_kinematic {
private:
    float m_alpha;
    float m_Curve_Radius;
    float m_Servo_Bearing_Radian;
    float m_Speed_Factor;
    float m_Velocity;
    float m_Angular_Velocity;
    float m_Left_M_Speed;
    float m_Right_M_Speed;
    bool m_Radius_Unobtainable;

public:
    robot_kinematic(float alpha);
    
    void cal(int Servo_Raw);
    float get_Servo_Bearing_Radian();
    float get_Radius();
    float get_Speed_factor();
    float get_velocity();
    float get_Angular_Velocity();
    float Left_M_Speed();
    float Right_M_Speed();
};


#endif