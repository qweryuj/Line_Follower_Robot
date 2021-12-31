#include "PD.h"



PD::PD(double Kp, double Kd) { // set Kp, Kd constant for this object
    m_Kp = Kp;
    m_Kd = Kd;
}


double PD::calculate(double error, double dt) { // calculate PD output
    m_error = (m_Kp*error) + (m_Kd*(error - m_error)/dt);
    return m_error;
}




