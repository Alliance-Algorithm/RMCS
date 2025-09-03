#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

namespace rmcs_core::controller::filter{

class MedianFilter{

public:
    MedianFilter()
    : n_(0)
    , new_output(0.0)
    {

    }
    void clear_noise(double i){
        if(n_==0){
            input[0] = i;
            new_output = i;
            n_++;
        }
        else if(n_==1){
            input[1] = i;
            n_++;
        }
        else if(n_==2){
            input[2]=i;
            n_++;
            new_output=sort_input();
        }
        else if(n_>2){
            input[0]=input[1];
            input[1]=input[2];
            input[2]=i;
            new_output=sort_input();
        }
    }

    double output(){
        return new_output;

    }
    double sort_input(){
        std::vector<double> sorted_datas;
        for(double j : input){
            sorted_datas.push_back(j);
        }
        sort(sorted_datas.begin(),sorted_datas.end());

        return sorted_datas[1];
    }
    int n_;
private:
    
    double new_output;
    double input[3];
    

};












}