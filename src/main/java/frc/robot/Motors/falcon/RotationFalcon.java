// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.falcon;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class RotationFalcon extends Falcon {
    double positonOffset;
    
    public RotationFalcon
    (
        TalonFX talon,
        int kPIDLoopIdx,
        double peakOutputForward, double peakOutputReverse,
        double Kf, double Kp, double Ki, double Kd,
        double positionOffset
    ) 
    {
        super(talon, kPIDLoopIdx, peakOutputForward, peakOutputReverse, Kf, Kp, Ki, Kd);
        this.positonOffset = positionOffset;
    }

    public void setAngle(double angle) {
        double ticksFromOffset = Math.min((angle / 360) * this.ticksForRotation, 360 - (angle / 360) * this.ticksForRotation);
        this.setPosition(positonOffset + ticksFromOffset);
    }
}
