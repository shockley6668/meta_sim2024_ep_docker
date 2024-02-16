#pragma
class PIDController 
{
private:
    double kp, ki, kd, threshold;
    double errorSum, lastError;

public:
    PIDController(double kp, double ki, double kd, double threshold) : kp(kp), ki(ki), kd(kd), threshold(threshold), errorSum(0), lastError(0) {}

    double calculate(double error, double dt = 0.01) 
    {
        errorSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kp * error + ki * errorSum + kd * derivative;

        if (output > threshold) {
            output = threshold;
        } else if (output < -threshold) {
            output = -threshold;
        }

        return output;
    }
};