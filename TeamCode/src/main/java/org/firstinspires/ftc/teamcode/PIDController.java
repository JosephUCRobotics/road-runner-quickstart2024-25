package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {

    public double Kp;
    public double Ki;
    public double Kd;
    double integralSum = 0;
    double lastError = 0;
    public double out;
    public double error;
    ElapsedTime timer = new ElapsedTime();
    public PIDController(double Kp_in, double Ki_in, double Kd_in) {
        Kp = Kp_in;
        Ki = Ki_in;
        Kd = Kd_in;
    }

    public void setPIDConstants(double Kp_in, double Ki_in, double Kd_in) {
        Kp = Kp_in;
        Ki = Ki_in;
        Kd = Kd_in;
    }
    public void resetTimer() {
        integralSum = 0;
        timer.reset();
    }
    double getError(){
        return error;
    }
    double getOut(){
        return out;
    }

    public double update(double target, double encoderPosition) {

        // calculate the error
        error = target - encoderPosition;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        // reset the timer for next time
        timer.reset();

        return out;
    }
}