// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;
import frc.robot.subsystems.BasicSwerveOdometry;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.coordinate;

public class PathFollower extends CommandBase {
  SwerveDriveTrain swerve;
  
  double Kp;
  double Kp_a;

  double[][] pathArray;

  StopWatch stopWatch = new StopWatch();
  double lastTime;

  BasicSwerveOdometry odometry;
  
  /** Creates a new PathFollower. */
  public PathFollower(double[][] pathArray, double Kp, double Kp_a, SwerveDriveTrain swerve, BasicSwerveOdometry odometry) {
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;
    this.Kp = Kp;
    this.pathArray = pathArray;

    this.odometry = odometry;
    lastTime = pathArray[pathArray.length - 1][0];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatch.start();
    System.out.println("starting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] currentPoint = pathArray[(int) Math.floor(stopWatch.getDuration() / 0.01)];
    coordinate robotCoordinate = odometry.getRobotCoordinate();

    Vector robotVector = new Vector(robotCoordinate.x, robotCoordinate.y, Representation.Cartisian);
    
    // [t, x_tag, y_tag, omega, x, y, angle]
    // [0,   1,    ,2,    ,3   ,4, 5,   6]

    System.out.println(String.format("wanted x: %f, wanted y: %f, wanted angle: %f", currentPoint[4], currentPoint[5], currentPoint[6]));
    System.out.println(String.format("x: %f, y: %f, angle: %f", robotVector.getX(), robotVector.getY(), odometry.getAngle()));

    Vector movementVector = new Vector(
      currentPoint[1] + (currentPoint[4] - robotVector.getX()) * Kp,
      currentPoint[2] + (currentPoint[5] - robotVector.getY()) * Kp,
      Representation.Cartisian
    );

    
    movementVector.rotateVector(Math.toRadians(90));

    System.out.println(String.format("x_tag: %f, y_tag: %f", movementVector.getX(), movementVector.getY()));

    
    swerve.setAbsoluteSwerveVelocoties(
      new Vector(movementVector.getX(), movementVector.getY(), Representation.Cartisian),     
      Math.toRadians(currentPoint[3]) + (odometry.getAngle() - currentPoint[6]) * Kp_a
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished!!!!!");
    swerve.setAbsoluteSwerveVelocoties(new Vector(0, 0, Representation.Polar), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.stopWatch.getDuration() > lastTime;
  }
}
