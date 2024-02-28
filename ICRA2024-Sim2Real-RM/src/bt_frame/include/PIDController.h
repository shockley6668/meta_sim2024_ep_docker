#pragma once
#include <vector>
class PIDController {
    public:
        PIDController(){}
        PIDController(double kp, double ki, double kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}
        PIDController(vector<double> param)
        {
            kp_ = param[0];
            ki_ = param[1];
            kd_ = param[2];
        }
        void clear_pid()
        {
            prev_error_ = 0;
            integral_ = 0;
        }
        double calculate(double setpoint, double now_measure) {
            double error = setpoint - now_measure;
            double p_out = kp_ * error;
            integral_ += error;
            double i_out = ki_ * integral_;
            double d_out = kd_ * (error - prev_error_);
            prev_error_ = error;
            double output = p_out + i_out + d_out;
            if(output<0.1&&output>0)
            {
                output = 0.1;
            }
            else if(output>-0.1&&output<0)
            {
                output = -0.1;
            }
            return output;
    }

    private:
        double kp_, ki_, kd_;
        double prev_error_;
        double integral_;
};