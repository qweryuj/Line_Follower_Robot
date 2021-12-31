#ifndef LINE_READ_CALIBRATE_H
#define LINE_READ_CALIBRATE_H

#include <mbed.h>
#include "math_function.h"
#include "constant_def.h"
#include "EWMA.h"

extern AnalogIn Sen_1;
extern AnalogIn Sen_2;
extern AnalogIn Sen_3;
extern AnalogIn Sen_4;
extern AnalogIn Sen_5;
extern AnalogIn Sen_6;

// extern DigitalOut On_board_LED;
extern DigitalOut On_Robot_LED;

extern DigitalIn SW_1; //note that on robot print as SW2
extern DigitalIn SW_2; //note that on robot print as SW3

extern EWMA Filter_1;
extern EWMA Filter_2;
extern EWMA Filter_3;
extern EWMA Filter_4;
extern EWMA Filter_5;
extern EWMA Filter_6;


class Line
{
private:
    double m_Sensor_Val[Num_of_Sensor]; // 6 = num of IR sensor
    double m_Line_mid_Val[Num_of_Sensor]; // mean of line and floor val
    double m_SD_Low_Pass;
    int m_Line_Pos;


public:
    Line(float alpha);

    void calibrate();
    void Print_Val(bool newline);
    void Print_SD(bool newline);
    void read(); // Read the value from sensor
    void Pos_Calculate(double SD_Low_Pass_Gain); // Calculate Line pos base on Sen_Val
    int Sen_Val(int i); // Read sensor value from array (specify index)
    int Line_mid_val(int i);
    int Line_Pos();
    
};



#endif