#ifndef Rpuffer_h
#define Rpuffer_h

#include "Arduino.h"



class Rpuffer
{
    public:
    Rpuffer(int);
    void set_NewValue(float);
    void clear();

    float get_NewValue();
    float get_Value(int);
    int get_Size();
    float get_Mean();

    private:  
    #define NvaluesMAX 20
    float Data[NvaluesMAX];
    int Ivalues = 0;
    int Nvalues = 0;

};
#endif