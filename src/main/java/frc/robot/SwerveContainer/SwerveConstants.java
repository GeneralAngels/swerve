// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class SwerveConstants {
    public static final double maxSpeed = 5;
    
    // Swerve gears and wheels: 
    public static final double wheelRadius = 0.0508;
    public static final double Driving_MotorToDrivenRatio = 8.14 / 1;
    public static final double Rotation_MotorToDrivenRatio = 150 / 7;

    public static final double swerveWidth = 0.6;
    public static final double swerveLength = 0.6;

    // Driving PID Constans: 
    public static final double Drive_Kf = 0.045;
    public static final double Drive_Kp = 0.06;
    public static final double Drive_Ki = 0;
    public static final double Drive_Kd = 0;

    // Rotation PID Constants:
    public static final double Rotation_Kf = 0;
    public static final double Rotation_Kp = 0.2;
    public static final double Rotation_Ki = 0.0;
    public static final double Rotation_Kd = 0.0;


    // Kinematics:
    // mod0: front right
    // mod1: right rear
    // mod2: left rear
    // mod3: left front
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(-SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2),
      new Translation2d(SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2),
      new Translation2d(SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2),
      new Translation2d(-SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2)
    );
    
    // CanCoder home angles:
    public static double homeFrontRightAngle = (37.001 + 180) % 360;
    public static double homeRearRightAngle = (224.38 + 180) % 360;
    public static double homeRearLeftAngle = (353.93 + 180) % 360;
    public static double homeFrontLeftAngle = (289.24 + 180) % 360;
}
