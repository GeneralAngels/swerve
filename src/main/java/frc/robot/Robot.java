// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Autonomous.PathFollower;
import frc.robot.Autonomous.PathTextParser;
import frc.robot.SwerveContainer.SwerveContainer;
import frc.robot.SwerveContainer.SwerveOdometry.WpilibOdometryWrapper;
import frc.robot.commands.LogCommand;
import frc.robot.commands.SwerveJoysticks;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();
  SwerveContainer swerveContainer = new SwerveContainer();

  PS4Controller controller = new PS4Controller(0);
  Command joysticksCommand = new SwerveJoysticks(controller, swerveContainer.swerve);

  WpilibOdometryWrapper wpilibOdometry = new WpilibOdometryWrapper(swerveContainer.swerve, new Pose2d(0, 0, new Rotation2d()));

  LogCommand log = new LogCommand(0, () -> {return 2.2;}, m_robotContainer.outputStream);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put ourP
    // autonomous chooser on the dashboard
  }

  @Override
  public void teleopPeriodic() {
    // joysticksCommand.execute();
    // coordinate coordinate = this.wpilibOdometry.getRobotCoordinate();
    // System.out.println(String.format("x: %f, y: %f", coordinate.x, coordinate.y));

    /*
    swerveContainer.swerve.setWpiRelativeSwerveVelocoties(
      new ChassisSpeeds(
        0.0,
        0.2,
        0.0
      )
    );
    */
    
    double angle = this.swerveContainer.gyro.getAngle();
    System.out.println(String.format("gyro angle: %f", Math.abs(angle) % 360 * -Math.signum(angle)));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    this.swerveContainer.swerve.gyro.reset();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    PathTextParser textParser = new PathTextParser(Filesystem.getDeployDirectory().getAbsolutePath() + "/Path.txt");
    // m_autonomousCommand = new PathFollower(textParser.getPathArray(), -0.0, 0.7, this.swerveContainer.swerve, new BasicSwerveOdometry(0, 0, 0, this.swerveContainer.swerve, this.swerveContainer.gyro));
    m_autonomousCommand = new PathFollower(
      textParser.getPathArray(), 
      0.5, 
      0.7, 
      this.swerveContainer.swerve, 
      wpilibOdometry
    );

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    swerveContainer.swerve.setDefaultCommand(joysticksCommand);
    new JoystickButton(controller, Button.kTriangle.value).whenActive(
      new StartEndCommand(
        () -> {this.swerveContainer.swerve.setWpiAbsoluteVelocoties(new ChassisSpeeds(0, 1, 0));}, 
        () -> {this.swerveContainer.swerve.setWpiAbsoluteVelocoties(new ChassisSpeeds(0, 0, 0));}, 
        swerveContainer.swerve)).whenInactive(new StartEndCommand(
          () -> {this.swerveContainer.swerve.setWpiAbsoluteVelocoties(new ChassisSpeeds(0, 0, 0));},
          () -> {}, 
          swerveContainer.swerve));

    new JoystickButton(controller, Button.kTouchpad.value).whenPressed(
      new FunctionalCommand(
        () -> {this.swerveContainer.swerve.resetAllEncoderToAbsolute();}, 
        () -> {}, 
        (Boolean isFinished) -> {}, 
        () -> {return true;}
        )
    );
  }

  /** This function is called periodically during operator control. */

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
