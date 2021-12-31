#ifndef MATH_FUCNTION_H
#define MATH_FUCNTION_H


class arr_SD_cal {
private:
    int m_size;
    double m_sum;
    double m_mean;
    double m_variance;
    double m_SD;

public:
    arr_SD_cal(int size);
    
    void cal(double arr[]);
    double getSum();
    double getMean();
    double getVarince();
    double getSD();
};


#endif