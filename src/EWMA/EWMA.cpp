#include "EWMA.h"

EWMA::EWMA() { // object created with alpha = 1
    m_alpha = 1;
}

EWMA::EWMA(float alpha) { // object created with specify alpha value
    m_alpha = alpha;
}


void EWMA::reset() { // reset EWMA Filter to give output=input once (useful when start new EWMA Filter after a period)
    m_Init = false;
}

void EWMA::set_alpha(float alpha) { // set alpha value
    m_alpha = alpha;
}

double EWMA::Filter(double Raw_Data) { // EWMA Filter with constant peroid of input
    if (m_Init == true) { 
        m_output = (m_alpha*Raw_Data) + ((1-m_alpha)*m_output); // second term and so forth
    }
    else {
        m_output = Raw_Data; // first term of EWMA calculation (Give output = input)
        m_Init = true; // stop this function after the first term
    }
    return m_output;
}