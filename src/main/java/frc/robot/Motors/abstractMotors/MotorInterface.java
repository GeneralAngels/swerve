// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.abstractMotors;

/** Add your docs here. */
public interface MotorInterface {
    public abstract void setRpm(double Rpm);
    public abstract void setPrecentage(double precentage);
    
    // public abstract void setAngle(double angle);
    public abstract void setPosition(double position);

    public abstract double getRpm();
    public abstract double getVelocityTicks();
}
