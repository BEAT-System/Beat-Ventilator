#include <Arduino.h>
#include "Rpuffer.h"

Rpuffer::Rpuffer(int N)
{
    Nvalues = N;
}

void Rpuffer::clear()
{
    Ivalues = 0;
    for(int i = 0; i < Nvalues; i++) {
        Data[i] = 0;
    }
}

void Rpuffer::set_NewValue(float NewData)
{
    if(Ivalues < Nvalues)
    {
        Data[Ivalues] = NewData;
        Ivalues =  Ivalues + 1;
    }
    else if(Ivalues >= Nvalues)
    {
        for(int i = 0; i <= Nvalues-2 ; i++) {
            Data[i] = Data[i+1];
        }
        Data[Nvalues-1] = NewData;

    }

}

float Rpuffer::get_NewValue()
{
    return Data[Ivalues-1];
}

float Rpuffer::get_Value(int I)
{
    if(I <= Ivalues && I>0){
       return Data[I-1]; 
    }
}

int Rpuffer::get_Size()
{
    return Ivalues;
}

float Rpuffer::get_Mean()
{
    float sum = 0.0;
    for(int i = 0; i<Ivalues; i++) {
        sum += Data[i];
    }
    return (sum/float(Ivalues));

}