// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveContainer;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import frc.robot.Motors.SwerveModule;
import frc.robot.Motors.falcon.Falcon;
import frc.robot.Motors.falcon.RotationFalcon;
import frc.robot.subsystems.SwerveDriveTrain;

/** Add your docs here. */
public class SwerveContainer {
    // Sensors:
    WPI_PigeonIMU gyro = new WPI_PigeonIMU(30);
    
    // Setting DrivingMotors:
    Falcon drivingRightFront = new Falcon(
        new TalonFX(11), 
        0, 
        0.7, -0.7, 
        SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
        SwerveConstants.Driving_MotorToDrivenRatio
    );

    Falcon drivingRightRear = new Falcon(
        new TalonFX(12), 
        0, 
        0.7, -0.7, 
        SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
        SwerveConstants.Driving_MotorToDrivenRatio
    );
    
    Falcon drivingLeftRear = new Falcon(
        new TalonFX(13), 
        0, 
        0.7, -0.7, 
        SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
        SwerveConstants.Driving_MotorToDrivenRatio
    );
    
    Falcon drivingLeftFront = new Falcon(
        new TalonFX(14), 
        0, 
        0.7, -0.7, 
        SwerveConstants.Drive_Kf, SwerveConstants.Drive_Kp, SwerveConstants.Drive_Ki, SwerveConstants.Drive_Kd,
        SwerveConstants.Driving_MotorToDrivenRatio
    );

    // Setting Rotation Motors:
    RotationFalcon rotationRightFront = new RotationFalcon(
      new TalonFX(21), 1, 
      0, 
      0.6, -0.6, 
      SwerveConstants.Rotation_MotorToDrivenRatio,
      SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Ki, SwerveConstants.Rotation_Kd, SwerveConstants.Rotation_Kd, 
      SwerveConstants.homeFrontRightAngle, 
      true, true
    );

    RotationFalcon rotationRightRear = new RotationFalcon(
      new TalonFX(22), 2,
      0, 
      0.6, -0.6, 
      SwerveConstants.Rotation_MotorToDrivenRatio,
      SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Ki, SwerveConstants.Rotation_Kd, SwerveConstants.Rotation_Kd, 
      SwerveConstants.homeFrontRightAngle, 
      true, true
    );
    
    RotationFalcon rotationLeftRear = new RotationFalcon(
      new TalonFX(23), 3, 
      0, 
      0.6, -0.6, 
      SwerveConstants.Rotation_MotorToDrivenRatio,
      SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Ki, SwerveConstants.Rotation_Kd, SwerveConstants.Rotation_Kd, 
      SwerveConstants.homeFrontRightAngle, 
      true, true
    );
    
    RotationFalcon rotationLeftFront = new RotationFalcon(
      new TalonFX(24), 4, 
      0, 
      0.6, -0.6, 
      SwerveConstants.Rotation_MotorToDrivenRatio,
      SwerveConstants.Rotation_Kf, SwerveConstants.Rotation_Ki, SwerveConstants.Rotation_Kd, SwerveConstants.Rotation_Kd, 
      SwerveConstants.homeFrontRightAngle, 
      true, true
    );

    SwerveModule moduleRightFront = new SwerveModule(drivingRightFront, rotationRightFront, SwerveConstants.wheelRadius);
    SwerveModule moduleRightRear = new SwerveModule(drivingRightRear, rotationRightRear, SwerveConstants.wheelRadius);
    SwerveModule modueLeftFront = new SwerveModule(drivingLeftFront, rotationLeftFront, SwerveConstants.wheelRadius);
    SwerveModule modueLeftRear = new SwerveModule(drivingLeftRear, rotationLeftRear, SwerveConstants.wheelRadius);

    public SwerveDriveTrain swerve = new SwerveDriveTrain(
        moduleRightFront, moduleRightRear, modueLeftRear, modueLeftFront, 
        gyro, 
        SwerveConstants.swerveWidth, SwerveConstants.swerveLength
    );
}
