#ifndef ASSIST_FUNCTION_H
#define ASSIST_FUNCTION_H

#include <stdio.h>
#include <stdint.h>

float my_abs(float x);

uint16_t my_abs_int(uint16_t x);

float constrain(float val,float min_val,float max_val){
    if(val>max_val) return max_val;
    if(val<min_val) return min_val;
    return val;
}

#endif