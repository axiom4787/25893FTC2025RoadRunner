package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double integral;
    private double previousError;
    private final ElapsedTime elapsedTime;

    private double previousTime;

    private double minOutput;
    private double maxOutput;

    public PIDController(double kP, double kI, double kD, double minOutput, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = 0;
        this.elapsedTime = new ElapsedTime();
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public double calculate(double setpoint, double actual) {
        double currentTime = elapsedTime.seconds();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        double error = setpoint - actual;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        // Clamp output between minOutput and maxOutput
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;
        return output;
    }

    public void reset() {
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = elapsedTime.seconds();
    }
}
