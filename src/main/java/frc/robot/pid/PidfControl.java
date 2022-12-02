// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pid;

import com.ctre.phoenix.time.StopWatch;

/** Add your docs here. */
public class PidfControl {
    double Kp;
    double Ki;
    double Kd;
    double Kf;

    double alphaConstant = 0.3;

    double filteredVelocity = 0;
    double previousPosition = 0;
    double previousVelocity = 0;

    double integral = 0;
    double maxIntegral = 1;
    double minSetpoint = 0;

    StopWatch stopWatch = new StopWatch();

    public PidfControl
    (
        double Kp, double Ki, double Kd, double Kf,
        double alpha

    ) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public void start() {
        stopWatch.start();
    }

    public void setKpid(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setMinMax(double minSetpoint, double maxIntegral) {
        this.minSetpoint = minSetpoint;
        this.maxIntegral = maxIntegral;
    }

    public double computeVelocity(double pos, Boolean isVelocity) {
        double rawVelocity;

        if (isVelocity) {
            rawVelocity = (pos - previousPosition) / (stopWatch.getDuration() - previousPosition);
            previousPosition = stopWatch.getDuration();
        }
        else {
            rawVelocity = pos;
        }

        double filteredVelocity = this.alphaFilter(rawVelocity, previousVelocity, alphaConstant);
        
        if (Math.abs(filteredVelocity) < 0.001) {
            filteredVelocity = 0;
        }

        return filteredVelocity;
    }

    public void computeIntegral(double error) {
        this.integral += error * this.Ki;
        this.integral = this.clamp(this.integral, -maxIntegral, maxIntegral);
    }

    public double positionControl(double setpoint, double pos) {
        // regular PID
        double filteredVelocity = this.computeVelocity(pos, false);

        double error = setpoint - pos;
        this.computeIntegral(error);
        return error * Kp + integral - Kd * filteredVelocity;
    }

    public double velocityControlByPosition(double setpoint, double pos) {
        // Calculate velocity from position!
        double filteredVelocity = this.computeVelocity(pos, false);
        double error = setpoint - filteredVelocity;
        this.computeIntegral(error);

        return error * Kp + this.integral + Kf * setpoint;
    }

    public static double clamp(double value, double minValue, double maxValue) {
        return Math.max(Math.min(value, maxValue), minValue);
    }
    
    public static double alphaFilter(double currentValue, double previousValue, double alpha) {
        return previousValue * alpha + currentValue * (1-alpha);
    }
}
