#pragma once
#include<cmath>

namespace rmcs_core::controller::filer{


class LowFilter{
public:
    LowFilter()
    : n_(0)
    , coefficient(0.1)
    , last_output(0.0)
    , new_input(0.0)
    , new_output(0.0)
    {}

    double clear_noise(double i){
        if(n_==0){
            new_output=i;
            last_output=new_output;
            n_++;
        }
        
        
        else if(n_>0)
        {
            new_input=i;
            new_output=coefficient * new_input + (1 - coefficient) * last_output;
            last_output=new_output;
            n_++;
    
        }

        return new_output;
    }


    void set_first_output(double l){
        last_output=l;
    }

    


private:
    int n_;
    double coefficient;
    double last_output;
    double new_input;
    double new_output;
};


}