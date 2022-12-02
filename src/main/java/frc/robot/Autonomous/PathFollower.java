// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import javax.crypto.Cipher;

import com.ctre.phoenix.time.StopWatch;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;
import frc.robot.subsystems.BasicSwerveOdometry;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.coordinate;

public class PathFollower extends CommandBase {
  SwerveDriveTrain swerve;
  double Kp;
  double[][] pathArray;

  StopWatch stopWatch = new StopWatch();

  BasicSwerveOdometry odometry;
  
  /** Creates a new PathFollower. */
  public PathFollower(double[][] pathArray, double Kp, SwerveDriveTrain swerve, BasicSwerveOdometry odometry) {
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;
    this.Kp = Kp;
    this.pathArray = pathArray;

    this.odometry = odometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] currentPoint = pathArray[(int)Math.floor(stopWatch.getDuration())];
    coordinate robotCoordinate = odometry.getRobotCoordinate();
    
    // [t, x_tag, y_tag, omega, x, y, angle]
    // [0,   1,    ,2,    ,3   ,4, 5,   6]
    swerve.setAbsoluteSwerveVelocoties(
      new Vector(
        currentPoint[1] + (currentPoint[4] - robotCoordinate.x) * Kp,
        currentPoint[2] + (currentPoint[5] - robotCoordinate.y) * Kp,
        Representation.Cartisian
      ),     
      currentPoint[3] + (currentPoint[6] - this.odometry.getAngle())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
