#include <math.h>
#include "math_function.h"
#include "constant_def.h"



arr_SD_cal::arr_SD_cal(int size) { // create object with specific size of array to process
    m_size = size;
}


void arr_SD_cal::cal(double *arr) { // calculate Sum, Mean, Variance, Standard deviation
    m_sum = 0.0;
    m_variance = 0.0;

    for(int i=0; i < m_size; i++) { // Sum
        m_sum += arr[i];
    }

    m_mean = m_sum/m_size; // Mean

    for(int i=0; i < m_size; i++) { // Variance
        m_variance += pow(arr[i] - m_mean, 2);
    }

    m_SD = sqrt(m_variance/m_size); // SD
}


double arr_SD_cal::getSum() { // return sum of the array
    return m_sum;
}


double arr_SD_cal::getMean() { // return mean of the array
    return m_mean;
}


double arr_SD_cal::getVarince() { // return variance of the array
    return m_variance;
}


double arr_SD_cal::getSD() { // return standard deviation of the array
    return m_SD;
}