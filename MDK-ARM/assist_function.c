#include "assist_function.h"

float my_abs(float x){
    return x>0.0f? x:-x;
}

uint16_t my_abs_int(uint16_t x){
    return x>0? x:-x;
}