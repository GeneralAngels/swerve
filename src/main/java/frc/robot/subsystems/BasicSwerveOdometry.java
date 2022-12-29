// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Vector3d;

public class BasicSwerveOdometry extends SubsystemBase {
  coordinate loc;

  StopWatch stopWatch = new StopWatch();
  double lastTime = 0;

  SwerveDriveTrain swerve;

  Gyro gyro;
  
  /** Creates a new BasicSwerveOdometry. */
  public BasicSwerveOdometry(double x0, double y0, double angle0, SwerveDriveTrain swerve, Gyro gyro) {
    this.loc = new coordinate(x0, y0, angle0);
    
    this.swerve = swerve;

    this.gyro = gyro;
  }

  public void update(double time, Vector3d robotVector) {
    this.loc.x += robotVector.getX() * time;
    this.loc.y += robotVector.getY() * time;
    this.loc.angle += robotVector.getOmega() * time;
  }

  public void start() {
    stopWatch.start();
  }

  public coordinate getRobotCoordinate() {
    return this.loc;
  }

  public double getAngle() {
    return -this.gyro.getAngle();
  }

  @Override
  public void periodic() {
    this.update(stopWatch.getDuration() - lastTime, this.swerve.getRobotVector());
    lastTime = stopWatch.getDuration();
    // This method will be called once per scheduler run
  }
}
