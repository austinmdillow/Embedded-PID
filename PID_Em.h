#ifndef PID_h
#define PID_h

class PID_Em {
    public:
        void PID(float* input, float* output, float* setpoint, double kp_in, double ki_in, double kd_in);
        void setTuning(double kp_in, double ki_in, double kd_in);

    private:

    double Kp, Ki, Kd;
};


#endif