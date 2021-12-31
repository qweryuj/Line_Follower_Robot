#ifndef PD_H
#define PD_H


class PD {
private:
    double m_error;
    double m_Kp, m_Kd;

public:
    PD(double Kp, double Kd);

    double calculate(double error, double dt);
};


#endif