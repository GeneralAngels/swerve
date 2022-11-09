// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.falcon;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class RotationFalcon extends Falcon {
    double positonOffset;
    double direction = 1;
    
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
        double ticksFromOffset = (angle / 360) * this.ticksForRotation;
        
        if (ticksFromOffset > this._talon.getSelectedSensorPosition(0)) {
            direction = -1;
        }
        else {
            direction = 1;
        }

        this.setPosition(positonOffset + ticksFromOffset);
    }

    @Override
    public void setRpm(double rpm) {
        super.setRpm(rpm * direction);
    }
}
