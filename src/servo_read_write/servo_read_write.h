#ifndef SERVO_READ_WRITE_H
#define SERVO_READ_WRITE_H

#include <mbed.h>
#include "EWMA.h"
#include "constant_def.h"


extern AnalogIn Servo_Poten;

extern EWMA Servo_Pos_Filter;


class Servo {
private:
    int m_Init_Servo_Pos; // 16 bit init pos
    float m_kp, m_kd; //Servo PD

public:
    Servo(float alpha); //set alpha value for the filter

    void Speed_Move(float speed); // move servo at 
    void Pos_Move(int pos, float kp, float kd);
    void stop(bool brake);
    int Servo_Init_Pos();
    int Servo_Pos();
};


#endif
