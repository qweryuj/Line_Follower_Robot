#include "line_read_calibrate.h"

AnalogIn Sen_1(Sen_1_Pin);
AnalogIn Sen_2(Sen_2_Pin);
AnalogIn Sen_3(Sen_3_Pin);
AnalogIn Sen_4(Sen_4_Pin);
AnalogIn Sen_5(Sen_5_Pin);
AnalogIn Sen_6(Sen_6_Pin);

DigitalOut On_board_LED(LED1);
DigitalOut On_Robot_LED(Robot_LED4);

DigitalIn SW_1(SW_1_Pin); //note that on robot print as SW3

EWMA Filter_1;
EWMA Filter_2;
EWMA Filter_3;
EWMA Filter_4;
EWMA Filter_5;
EWMA Filter_6;

arr_SD_cal Line_SD(Num_of_Sensor);



Line::Line(float alpha) { // defualt constructor, setting alpha value for all sensor filter
    Filter_1.set_alpha(alpha);
    Filter_2.set_alpha(alpha);
    Filter_3.set_alpha(alpha);
    Filter_4.set_alpha(alpha);
    Filter_5.set_alpha(alpha);
    Filter_6.set_alpha(alpha);
}


void Line::calibrate() { //Calibrate and calculate mid val
    int Line_Val[Num_of_Sensor] = {0,0,0,0,0,0};
    int Floor_Val[Num_of_Sensor] = {0,0,0,0,0,0};

    On_Robot_LED.write(1); //turn on Robot_LED
    while(SW_1.read() == 0); //wait until button has push

    for(int i=0; i<=20; i++) { //reading the line 20 time
        Line_Val[0] += Sen_1.read_u16();
        Line_Val[1] += Sen_2.read_u16();
        Line_Val[2] += Sen_3.read_u16();
        Line_Val[3] += Sen_4.read_u16();
        Line_Val[4] += Sen_5.read_u16();
        Line_Val[5] += Sen_6.read_u16();
        wait_us(50000);
        On_Robot_LED.write(!On_Robot_LED.read());
    }
    printf("Line_Val = "); //print Line_Val
    for(int i = 0; i < Num_of_Sensor; i++) {
        printf(", %d",int(Line_Val[i]/20));
    }
    printf("\n");

    On_Robot_LED.write(1); // turn on Robot_LED
    while(SW_1.read() == 0); // wait until button has push

    for(int i=0; i<=20; i++) { // reading the floor 20 time
        Floor_Val[0] += Sen_1.read_u16();
        Floor_Val[1] += Sen_2.read_u16();
        Floor_Val[2] += Sen_3.read_u16();
        Floor_Val[3] += Sen_4.read_u16();
        Floor_Val[4] += Sen_5.read_u16();
        Floor_Val[5] += Sen_6.read_u16();
        wait_us(50000);
        On_Robot_LED.write(!On_Robot_LED.read());
    }
    printf("Floor_Val = "); //print Floor_Val
    for(int i = 0; i < Num_of_Sensor; i++) {
        printf(", %d",int(Floor_Val[i]/20));
    }
    printf("\n");

    for(int i=0; i<Num_of_Sensor; i++) { //calculate value between floor and line (mid val)
        m_Line_mid_Val[i] = (Line_Val[i] + Floor_Val[i])/40.0;
        m_SD_Low_Pass += abs(Line_Val[i] - Floor_Val[i])/(40.0*6.0); // calculate expected SD during normal operation
    }

    On_Robot_LED.write(0); //turn off the LED indicate the end of setup process

    printf("Line_mid_Val = "); //print m_Line_mid_Val
    for(int i = 0; i < Num_of_Sensor; i++) {
        printf(", %d",int(m_Line_mid_Val[i]));
    }
    printf("      \n");
    printf("SD_Low_pass = %d", int(m_SD_Low_Pass));
    while(SW_1.read() == 0); // wait for the exit
}


void Line::read() { // Read the value from sensor, then, subtracted by mid val
    m_Sensor_Val[0] = Filter_1.Filter(Sen_1.read_u16()) - m_Line_mid_Val[0];
    m_Sensor_Val[1] = Filter_2.Filter(Sen_2.read_u16()) - m_Line_mid_Val[1];
    m_Sensor_Val[2] = Filter_3.Filter(Sen_3.read_u16()) - m_Line_mid_Val[2];
    m_Sensor_Val[3] = Filter_4.Filter(Sen_4.read_u16()) - m_Line_mid_Val[3];
    m_Sensor_Val[4] = Filter_5.Filter(Sen_5.read_u16()) - m_Line_mid_Val[4];
    m_Sensor_Val[5] = Filter_6.Filter(Sen_6.read_u16()) - m_Line_mid_Val[5];
}


void Line::Pos_Calculate(double SD_Low_Pass_Gain) { // Calculate Line pos base on Sen_Val
    double Line_Pos = 0;
    int Num_of_Line_Detected = 0;
    Line_SD.cal(m_Sensor_Val); // calculate SD value from m_Sensor_Val array

    // compare expected operation SD value with robot process SD value
    // factor(SD_Gain) = (1 - acceptable range(in percent))
    if (Line_SD.getSD() > SD_Low_Pass_Gain*m_SD_Low_Pass) { // SD passing threshold indicate that the robot see the line
        On_Robot_LED.write(0); // turn off the LED
        if(Line_SD.getSum() > 0) { // checking whether line val or floor val were greater
            for(int i = 0; i < Num_of_Sensor; i++) { // Line val < floor val, Line counting
                if(m_Sensor_Val[i] < 0) { 
                    Num_of_Line_Detected++;
                    Line_Pos += ((i+1));
                }
            }
        }
        else {
            for(int i = 0; i < Num_of_Sensor; i++) { // Floor val < Line val, Line counting
                if(m_Sensor_Val[i] > 0) { 
                    Num_of_Line_Detected++;
                    Line_Pos += ((i+1));
                }
            }
        }
    }
    else {
        On_Robot_LED.write(1); // turn on the LED which indicate error
        Line_Pos = 0;  // SD is too low, all sensor get the same value
    }


    if (Num_of_Line_Detected == 0) { // zero division guard
        Line_Pos = 0; 
    }
    else {
        Line_Pos = Line_Pos*10.0/Num_of_Line_Detected; // Calculate Line position
    }

    m_Line_Pos = int(Line_Pos); //Store Line pos in class variable
}

int Line::Line_Pos() { //return current line position
    return m_Line_Pos;
}

int Line::Sen_Val(int i) { // return current sensor value from array (specify index number)
    return m_Sensor_Val[i];
}


int Line::Line_mid_val(int i) { // return Line_mid_Val from array (specify index number)
    return m_Line_mid_Val[i];
}

void Line::Print_SD(bool newline) { // Print SD value
    printf("SD = %d", int(Line_SD.getSD()));
    if(newline) {
        printf("\n");
    }
}

void Line::Print_Val(bool newline) { // Print m_Sensor_Val
    printf("Sen_Val = ");
    for(int i = 0; i < Num_of_Sensor; i++) {
        printf(", %d",int(m_Sensor_Val[i]));
    }
    if(newline) {
        printf("\n");
    }
}