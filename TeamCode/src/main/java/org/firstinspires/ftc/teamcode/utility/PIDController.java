package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class PIDController {
    private final double Kp, Ki, Kd;
    private final ElapsedTime timer;

    private double lastError, integralSum;
    private boolean isFirstIteration;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        timer = new ElapsedTime();
        reset();
    }

    public double calculate(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        double output;

        if (isFirstIteration) {
            output = Kp * error;
            lastError = error;
            timer.reset();
            isFirstIteration = false;
        } else {
            // Rate of change of the error
            double deltaTime = timer.seconds();
            double derivative = deltaTime != 0.0 ? (error - lastError) / deltaTime : 0.0;

            // Sum of all error over time
            integralSum = integralSum + error * deltaTime;

            output = Kp * error + Ki * integralSum + Kd * derivative;
            lastError = error;

            // Reset the timer for the next time
            timer.reset();
        }

        return output;
    }

    public void reset() {
        lastError        = 0.0;
        integralSum      = 0.0;
        isFirstIteration = true;
    }
}