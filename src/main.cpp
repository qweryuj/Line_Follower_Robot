#include <cmath>
#include <mbed.h>
#include "line_read_calibrate.h"
#include "servo_read_write.h"
#include "PD.h"
#include "kinematic.h"

Timer t_1;
Timer t_2;


Servo main_Servo(0.1);
Line main_Line(0.1);
PD Servo_PD(0.03, 0.1);


robot_kinematic kinematic(10.0f);


PwmOut R_Motor_PWM(R_Motor_PWM_Pin);
DigitalOut R_Motor_Sig(R_Motor_Sig_Pin);
PwmOut L_Motor_PWM(L_Motor_PWM_Pin);
DigitalOut L_Motor_Sig(L_Motor_Sig_Pin);


void Line_Setup();
void Serial_Print(int Time_Interval);
void Line_track();


unsigned long long dt_1;
unsigned long long dt_2; // microsecond


        // main_Servo.Speed_Move(0.3); //servo rotate at x speed
        // Servo_Pos = main_Servo.Servo_Init_Pos(); // Servo Position reading
        // End test 

int main() {
    Line_Setup();

    while(1) { // main loop
        t_1.reset();
        dt_1 = std::chrono::duration_cast<std::chrono::microseconds>(t_1.elapsed_time()).count();
        
        Serial_Print(1000000);
        /*
        R_Motor_PWM.write(0.2f);
        R_Motor_Sig.write(0.0f);
        
        L_Motor_PWM.write(0.2f);
        L_Motor_Sig.write(0.0f);

        thread_sleep_for(1000);


        R_Motor_PWM.write(0.0f);
        R_Motor_Sig.write(0.0f);
        
        L_Motor_PWM.write(0.0f);
        L_Motor_Sig.write(0.0f);


        while(SW_1.read() == 0);
        thread_sleep_for(1000);
        */
        
        main_Line.read(); // Read line position
        main_Line.Pos_Calculate(0.7);

        if(main_Line.Line_Pos() == 0) { // Determine wether the line is readable or not (0 = unreadable)
            main_Servo.stop(false); //Unreadable path
            R_Motor_PWM.write(0.0f); //stop the robot
            R_Motor_Sig.write(0.0f);
            L_Motor_PWM.write(0.0f);
            L_Motor_Sig.write(0.0f); 

        }
        else {
            main_Servo.Speed_Move(float(Servo_PD.calculate((35-main_Line.Line_Pos()*Sen_Mirroring), dt_1))); //move the arm
            kinematic.cal(main_Servo.Servo_Pos()); //calculate the left and right motor speed
            
            L_Motor_Sig.write(0.0f); //move the robot
            R_Motor_PWM.write(kinematic.Right_M_Speed());
            L_Motor_Sig.write(0.0f);
            L_Motor_PWM.write(kinematic.Left_M_Speed());
        }

    }
}



void Line_Setup(){
    R_Motor_PWM.write(0.0f); //stop the robot
    R_Motor_Sig.write(0.0f);

    L_Motor_PWM.write(0.0f);
    L_Motor_Sig.write(0.0f);

    main_Line.calibrate(); //Setup (SW2)
    t_1.start(); //Timer start
    t_2.start();
    printf("\nit working\n");
    thread_sleep_for(1000);
}


void Serial_Print(int Time_Interval){
    dt_2 = std::chrono::duration_cast<std::chrono::microseconds>(t_2.elapsed_time()).count(); //read timerjj
    if(dt_2 > Time_Interval){
        t_2.reset(); // Reset Timer
        printf("Radius = %d", int(kinematic.get_Radius()*1000)); //change  m to mm by *1000
        printf("    , Speed_factor = %d", int(kinematic.get_Speed_factor()*1000));
        printf("    , Servo_Angle = %d", int(kinematic.get_Servo_Bearing_Radian()*100));
        printf("    , Velocity = %d", int(kinematic.get_velocity()*100)); // convert from decimal to percent
        printf("    , Angualar_V = %d", int(kinematic.get_Angular_Velocity()*100));
        printf("    , Motor_Speed = %d", int(kinematic.Left_M_Speed()*100));
        printf("    , Motor_Speed = %d", int(kinematic.Right_M_Speed()*100));
        // printf("servo_raw = %d", main_Servo.Servo_Pos());
        printf("\n");
    }
}









/*

calibrate IR Sensor(measure Floor and line value)
calibrate Servo (move with hand to mid position)
  wait for signal --> 
  {
    line read (Finish)
    check for line end --> stop robot (finish)
    check for blank line --> stop robot (finish)
    calculate error (finish)
    calculate PD output (finish)
    move servo accordingly (finish)

    calculate servo position (finish)
    calculate error (finish)
    Kinematic (in process)
    determine motor speed
    calculate PD output
    calculate robot movement
    move the robot
    tuning the robot
  }

*/