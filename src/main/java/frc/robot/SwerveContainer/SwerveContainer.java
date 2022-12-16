// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveContainer;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Motors.falcon.Falcon;

/** Add your docs here. */
public class SwerveContainer {
    // Setting DrivingMotors:
    Falcon drivingRightFront = new Falcon(
        new TalonFX(11), 
        0, 
        0.7, -0.7, 
        SwerveConstants.Kf, SwerveConstants.Kp, SwerveConstants.Ki, SwerveConstants.Kd,
        SwerveConstants.MotorToDrivenRatio
    );

    Falcon drivingRightRear;
    Falcon drivingLeftRear;
    Falcon drivingLeftFront;
}
