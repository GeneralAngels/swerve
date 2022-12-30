// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveContainer.SwerveOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveContainer.SwerveConstants;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.coordinate;

public class WpilibOdometryWrapper extends SubsystemBase {
  SwerveDriveTrain swerve;
  SwerveDriveOdometry wpilibOdometry;

  Pose2d robotLastPosition;
  
  /** Creates a new WpilibOdometryWrapper. */
  public WpilibOdometryWrapper(SwerveDriveTrain swerve, Pose2d initalPose) {
    this.swerve = swerve;
    
    // Setting structure of swerve:
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2),
      new Translation2d(SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2),
      new Translation2d(-SwerveConstants.swerveWidth / 2, -SwerveConstants.swerveLength / 2),
      new Translation2d(-SwerveConstants.swerveWidth / 2, SwerveConstants.swerveLength / 2)
    );

    // Setting odometry:
    this.wpilibOdometry = new SwerveDriveOdometry(
      kinematics, 
      Rotation2d.fromDegrees(this.swerve.gyro.getAngle()),
      initalPose
    );
  }
  
  public void update() {
    robotLastPosition = this.wpilibOdometry.update(
      Rotation2d.fromDegrees(this.swerve.gyro.getAngle()),
      this.swerve.getRightFrontModuleState(),
      this.swerve.getRightRearModuleState(),
      this.swerve.getLeftRearModuleState(),
      this.swerve.getLeftFrontModuleState()
    );
  }

  public coordinate getRobotCoordinate() {
    Vector robotPosition = new Vector(robotLastPosition.getX(), robotLastPosition.getY(), Representation.Cartisian);
    robotPosition.rotateVector(Math.toRadians(-90));
    
    return new coordinate(robotPosition.getX(), -robotPosition.getY(), -this.swerve.gyro.getAngle());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.update();
  }
}
