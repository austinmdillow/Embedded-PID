#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

#include <PID_Em.h>

// TODO add error checking 

void PID_Em::PID(float* input, float* output, float* setpoint_in, double kp_in, double ki_in, double kd_in) {
    last_compute = millis();
    setTuning(kp_in, ki_in, kd_in);
    setOutputContraints(0, 255); // limits of Arduino DAC
}

void PID_Em::compute() {
    double dt = millis() - last_compute;
    if (dt >= compute_period) {
        // do the computation
        dt = dt / 1000; // convert delta T to seconds
        double error = setpoint - *pidInput;
        integral_sum += error * dt;
        double derrivative = (error - last_error) / dt;
        double result = error * kp + integral_sum * ki + derrivative * kd;


        // Check that the result is within the give contraints
        if (result > maxOut) {
            result = maxOut;
        } else if (result < minOut) {
            result = minOut;
        }

        *pidOutput = result;

        last_error = error;
        last_compute = millis();
        
    }
}

void PID_Em::setTuning(double kp_in, double ki_in, double kd_in) {
    kp = kp_in;
    ki = ki_in;
    kd = kd_in;
}

void PID_Em::setComputePeriod(int p) { ///< Period in ms between computes
    if (p > 0) {
        compute_period = p;
    }
}

void PID_Em::setOutputContraints(double min, double max) {
    minOut = min;
    maxOut = max;
}

void PID_Em::newSetpoint(double setpoint_in) {
    setpoint = setpoint_in;
    last_compute = millis();
    // TODO alleviate problems of high derrivative controller
    integral_sum = 0; // reset the integral controller
}

void PID_Em::printTuning() {
    Serial.print("Kp = ");
    Serial.println(kp);
    Serial.print("Ki = ");
    Serial.println(ki);
    Serial.print("Kd = ");
    Serial.println(kd);
}

double PID_Em::getKp() {
    return kp;
}

double PID_Em::getKi() {
    return ki;
}

double PID_Em::getKd() {
    return kd;
}