#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

#include <PID_Em.h>
#include <math.h>

// TODO add error checking 

PID_Em::PID_Em(double* input, double* output, double* setpoint_in, double kp_in, double ki_in, double kd_in) {
    last_compute = millis();
    setTuning(kp_in, ki_in, kd_in);
    setOutputContraints(0, 255); // limits of Arduino DAC
}

/**
 * Perform the PID calculation and sets output value.
 * This should be called in a loop and more frequently than the update period
 */
void PID_Em::compute() {
    double dt = millis() - last_compute;
    if (dt >= compute_period) {
        if (dt >= 1.5 * compute_period) {
            error_codes = PID_Stall;
        }
        // do the computation
        dt = dt / 1000; // convert delta T to seconds
        double error = setpoint - *pidInput;
        
        // checks for integral windup
        if (anti_windup) {
            if (fabs(error) > integral_domain) {
                integral_sum += error * dt;
                if (integral_sum > integral_max) {
                    integral_sum = integral_max;
                } else if (integral_sum < -integral_max) {
                    integral_sum = -integral_max;
                }
            }
        } else {
            integral_sum += error * dt;
        }

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

/**
 * Sets the time between PID computations
 * @param p time in ms
 */
void PID_Em::setComputePeriod(int p) { ///< Period in ms between computes
    if (p > 0) {
        compute_period = p;
    }
}

/**
 * Constrains the output of the controller to the given range
 * @param min the lower bound
 * @param max the upper bound
 */
void PID_Em::setOutputContraints(double min, double max) {
    minOut = min;
    maxOut = max;
}

/**
 * Controls the anit-windup portion of the controller.
 * @param maxIntegral how large the integral part can become in the output range
 * @param integrationDomain how far (either side of the setpoint) the integration component will continue to add
 */
void PID_Em::setIntegralWindup(double maxIntegral, double integrationDomain) {
    integral_max = maxIntegral;
    integral_domain = integrationDomain;
    anti_windup = true;
}

void PID_Em::resetIntegralWindup() {
    anti_windup = false;
}

void PID_Em::newSetpoint(double setpoint_in) {
    if (setpoint != setpoint_in) { // if there is no change, do nothing
        setpoint = setpoint_in;
        last_compute = millis();
        // TODO alleviate problems of high derrivative controller
        integral_sum = 0; // reset the integral controller
    }
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