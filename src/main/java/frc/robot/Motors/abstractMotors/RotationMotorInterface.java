// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.abstractMotors;

/** Add your docs here. */
public interface RotationMotorInterface extends MotorInterface {
    public void setAngle(double angle);
    public double getAngleByCanCoder();
    public void setEncoder();
    public double getAngleByFalcon();
    public void changeFlipped();
}
