#ifndef PID_h
#define PID_h

class PID_Em {
public:
  PID_Em(double* input, double* output, double* setpoint_in, double kp_in, double ki_in, double kd_in);
  void compute();
  void printTuning();
  void newSetpoint(double setpoint_in);
  void setTuning(double kp_in, double ki_in, double kd_in);
  void setIntegralWindup(double maxIntegral, double integrationDomain);
  void resetIntegralWindup();
  void setComputePeriod(int p);
  void setOutputContraints(double min, double max);
  double getKp(); //return the current Kp constant
  double getKi(); //return the current Ki constant
  double getKd(); //return the current Kd constant

  enum Errors {
    PID_Stall
  };

private:
  
  Errors error_codes;
  double kp, ki, kd;
  double setpoint;
  bool anti_windup = false; // controls if we implement integral anti-windup tactics
  double integral_sum, integral_domain, integral_max; // store the integral portion of the controller
  int compute_period = 50; // how often the PID controller computes (if called more frequently)
  double last_input, last_output, last_error;
  unsigned long last_compute;
  double *pidInput;
  double *pidOutput;
  double maxOut, minOut;
};

#endif