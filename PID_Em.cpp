#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

#include <PID_Em.h>

void PID_Em::PID(float* input, float* output, float* setpoint, double kp_in, double ki_in, double kd_in) {

}

void PID_Em::setTuning(double kp_in, double ki_in, double kd_in) {
    
}