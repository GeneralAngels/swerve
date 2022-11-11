// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Motors.falcon.Falcon;
import frc.robot.Motors.falcon.RotationFalcon;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  TalonFX motor;
  CANCoder canCoder = new CANCoder(1);

  private RobotContainer m_robotContainer;
  
  RotationFalcon frontRight;
  RotationFalcon rearRight;
  RotationFalcon rearLeft;
  RotationFalcon frontLeft;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public void printEncoders() {
    CANCoder rightFrontEncoder = new CANCoder(1);
    System.out.println("right front encoder: " + rightFrontEncoder.getAbsolutePosition());

    CANCoder rightRearEncoder = new CANCoder(2);
    System.out.println("right rear encoder: " + rightRearEncoder.getAbsolutePosition());

    CANCoder leftRearEncoder = new CANCoder(3);
    System.out.println("left rear encoder: " + leftRearEncoder.getAbsolutePosition());

    CANCoder leftFrontEncoder = new CANCoder(4);
    System.out.println("left front encoder: " + leftFrontEncoder.getAbsolutePosition());
  }
  
   @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    printEncoders();
    
    motor = new TalonFX(21);
    frontRight = new RotationFalcon(
      motor, 1, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeFrontRightAngle, 
      true, true
    );
    frontRight.setFalconEncoder();

    motor = new TalonFX(22);
    rearRight = new RotationFalcon(
      motor, 2, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeRearRightAngle, 
      true, true
    );
    rearRight.setFalconEncoder();

    motor = new TalonFX(23);
    rearLeft = new RotationFalcon(
      motor, 3, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeRearLeftAngle, 
      true, true
    );
    rearLeft.setFalconEncoder();

    motor = new TalonFX(24);
    frontLeft = new RotationFalcon(
      motor, 4, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeFrontLeftAngle, 
      true, true
    );
    frontLeft.setFalconEncoder();
    
  }

  @Override
  public void teleopPeriodic() {
    frontRight.setAngle(0);
    rearRight.setAngle(0);
    rearLeft.setAngle(0);
    frontLeft.setAngle(0);
    // System.out.println(falcon.getPosition());
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
    // System.out.println("falcon encoder: " + motor.getSelectedSensorPosition());
    // System.out.println("canCoder: " + canCoder.getAbsolutePosition() + ", falcon encoder: " + falcon.getPosition());

    // System.out.println(leftFrontRotation.getSelectedSensorPosition());
    // System.out.println(rightFrontEncoder.getValue());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
