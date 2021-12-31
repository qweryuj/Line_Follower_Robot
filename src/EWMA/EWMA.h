#ifndef EWMA_H
#define EWMA_H

#include "math_function.h"


class EWMA {
private:
    double m_output;
    double m_alpha = 1;
    bool m_Init = false;

public:
    EWMA();
    EWMA(float alpha); // Object created with Intial alpha

    double Filter(double Raw_Data); //EWMA Filter with constant peroid of input
    void reset(); //reset EWMA Filter to give output=input once (useful when start new EWMA Filter)
    void set_alpha(float alpha); //set alpha value
};


#endif