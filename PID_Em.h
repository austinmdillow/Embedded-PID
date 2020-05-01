#ifndef PID_h
#define PID_h

class PID_Em {
public:
  PID_Em(float* input, float* output, float setpoint_in, float kp_in, float ki_in, float kd_in);
  void compute();
  void printTuning();
  void newSetpoint(float setpoint_in);
  void setTuning(float kp_in, float ki_in, float kd_in);
  void setIntegralWindup(float maxIntegral, float integrationDomain);
  void resetIntegralWindup();
  void setComputePeriod(int p);
  void setOutputContraints(float min, float max);
  float getKp(); //return the current Kp constant
  float getKi(); //return the current Ki constant
  float getKd(); //return the current Kd constant

  enum Errors {
    PID_Stall
  };

private:
  
  Errors error_codes;
  float kp, ki, kd;
  float setpoint;
  bool anti_windup = false; // controls if we implement integral anti-windup tactics
  float integral_sum, integral_domain, integral_max; // store the integral portion of the controller
  int compute_period = 50; // how often the PID controller computes (if called more frequently)
  float last_input, last_output, last_error;
  unsigned long last_compute;
  float *pidInput;
  float *pidOutput;
  float maxOut, minOut;
};

#endif