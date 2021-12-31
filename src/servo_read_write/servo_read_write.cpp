#include "servo_read_write.h"



AnalogIn Servo_Poten(Servo_Poten_Pin);
PwmOut Servo_In_1(Servo_In_1_Pin);
PwmOut Servo_In_2(Servo_In_2_Pin);

EWMA Servo_Pos_Filter;


Servo::Servo(float alpha) {
    Servo_Pos_Filter.set_alpha(alpha); //set alpha value for the filter
    Servo_In_1.write(0.0f); //stop servo motor
    Servo_In_2.write(0.0f);
}


void Servo::Speed_Move(float speed) {
    m_Init_Servo_Pos = Servo_Pos_Filter.Filter(Servo_Poten.read_u16()); // Read Servo position
    if(speed == 0) { //stop
        Servo_In_1.write(0.0f);
        Servo_In_2.write(0.0f);
    }
    else if(speed > 0) {
        if(m_Init_Servo_Pos < Servo_Upper_Limit) {
            Servo_In_1.write(0.0f);
            Servo_In_2.write(abs(speed)); //right
        }
        else {
            Servo_In_1.write(0.0f); //beyond working area (stop)
            Servo_In_2.write(0.0f);
        }
    }
    else {
        if(m_Init_Servo_Pos > Servo_Lower_Limit) {
            Servo_In_1.write(abs(speed)); //left
            Servo_In_2.write(0.0f);
        }
        else {
            Servo_In_1.write(0.0f); //beyond working area (stop)
            Servo_In_2.write(0.0f);
        }
    }
}


void Servo::stop(bool brake) {
    if(brake == true) {
        Servo_In_1.write(1.0f); //e-brake
        Servo_In_2.write(1.0f);
    }
    else {
        Servo_In_1.write(0.0f); //stop
        Servo_In_2.write(0.0f);
    }
}


int Servo::Servo_Init_Pos() {
    return m_Init_Servo_Pos;
}


int Servo::Servo_Pos() {
    int Servo_Pos = Servo_Pos_Filter.Filter(Servo_Poten.read_u16());
    return Servo_Pos;
}