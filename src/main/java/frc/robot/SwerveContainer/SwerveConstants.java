// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveContainer;

/** Add your docs here. */
public class SwerveConstants {
    // Swerve gears and wheels: 
    public static final double wheelRadius = 0.0508;
    public static final double MotorToDrivenRatio = 8.14 / 1;

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


    // CanCoder home angles:
    public static double homeFrontRightAngle = 37.001;
    public static double homeRearRightAngle = 224.38;
    public static double homeRearLeftAngle = 353.93;
    public static double homeFrontLeftAngle = 289.24;
}
