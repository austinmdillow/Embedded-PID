#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

#include <PID_Em.h>

void PID_Em::PID(float* input, float* output, float* setpoint_in, double kp_in, double ki_in, double kd_in) {
    last_compute = millis();
    setTuning(kp_in, ki_in, kd_in);
}

void PID_Em::compute() {
    float dt = millis() - last_compute;
    if (dt >= compute_period) {
        // do the computation
        double error = setpoint - *pidInput;
        integral_sum += error * dt;
        double result = error * kp + integral_sum * ki + (error - last_error);

        last_input = *pidInput;
        last_error = error;
        last_compute = millis();
        *pidOutput = result;
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

void PID_Em::newSetpoint(double setpoint_in) {
    setpoint = setpoint_in;
    last_compute = millis();
}

void PID_Em::printTuning() {
    Serial.print("Kp = ");
    Serial.println(kp);
    Serial.print("Ki = ");
    Serial.println(ki);
    Serial.print("Kd = ");
    Serial.println(kd);
}