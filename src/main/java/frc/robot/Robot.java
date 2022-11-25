// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Motors.SwerveModule;
import frc.robot.Motors.falcon.Falcon;
import frc.robot.Motors.falcon.RotationFalcon;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;
import frc.robot.commands.ControllerCalculator;
import frc.robot.commands.LogCommand;
import frc.robot.commands.SwerveJoysticks;
import frc.robot.subsystems.SwerveDriveTrain;

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

  private RobotContainer m_robotContainer = new RobotContainer();
  
  RotationFalcon rotationRightFront;
  RotationFalcon rotationRightRear;
  RotationFalcon rotationLeftRear;
  RotationFalcon rotationLeftFront;

  Falcon drivingRightFront;
  Falcon drivingRightRear;
  Falcon drivingLeftRear;
  Falcon drivingLeftFront;

  SwerveModule moduleRightFront;
  SwerveModule moduleRightRear;
  SwerveModule moduleLeftFront;
  SwerveModule moduleLeftRear;
  SwerveDriveTrain swerve;
  SwerveJoysticks joystick;
  ControllerCalculator calculator;
  PS4Controller controller;

  LogCommand log = new LogCommand(0, () -> {return 2.2;}, m_robotContainer.outputStream);

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
    
    System.out.println("scheduled: " + log.isScheduled());

    printEncoders();
    
    motor = new TalonFX(21);
    rotationRightFront = new RotationFalcon(
      motor, 1, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeFrontRightAngle, 
      true, true
    );
    rotationRightFront.setFalconEncoder();

    motor = new TalonFX(22);
    rotationRightRear = new RotationFalcon(
      motor, 2, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeRearRightAngle, 
      true, true
    );
    rotationRightRear.setFalconEncoder();

    motor = new TalonFX(23);
    rotationLeftRear = new RotationFalcon(
      motor, 3, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeRearLeftAngle, 
      true, true
    );
    rotationLeftRear.setFalconEncoder();

    motor = new TalonFX(24);
    rotationLeftFront = new RotationFalcon(
      motor, 4, 
      0, 
      0.6, -0.6, 
      0, 0.2, 0.0, 0.0, 
      0, Constants.SwerveConstants.homeFrontLeftAngle, 
      true, true
    );
    rotationLeftFront.setFalconEncoder();
    
    drivingRightFront = new Falcon(
      new TalonFX(11), 
      0, 
      0.7, -0.7, 
      0.045, 0.06, 0, 0,
      8.14
    );

    drivingRightRear = new Falcon(
      new TalonFX(12), 
      0, 
      0.7, -0.7, 
      0.045, 0.06, 0, 0,
      8.14
    );

    drivingLeftRear = new Falcon(
      new TalonFX(13), 
      0, 
      0.7, -0.7, 
      0.045, 0.06, 0, 0,
      8.14
    );

    drivingLeftFront = new Falcon(
      new TalonFX(14), 
      0, 
      0.7, -0.7, 
      0.045, 0.06, 0, 0,
      8.14
    );
    moduleRightFront = new SwerveModule(drivingRightFront, rotationRightFront, 
                                        1 / (2 * Math.PI * 0.19) * 60);
    moduleRightRear = new SwerveModule(drivingRightRear, rotationRightRear, 
                                        1 / (2 * Math.PI * 0.19) * 60);
    moduleLeftFront = new SwerveModule(drivingLeftFront, rotationLeftFront, 
                                        1 / (2 * Math.PI * 0.19) * 60);
    moduleLeftRear = new SwerveModule(drivingLeftRear, rotationLeftRear, 
                                        1 / (2 * Math.PI * 0.19) * 60);
    
    swerve = new SwerveDriveTrain(
      moduleRightFront, moduleRightRear, moduleLeftRear, moduleLeftFront, 
      new WPI_PigeonIMU(30),
      0.7, 0.7
    );
    
    controller = new PS4Controller(0);
    joystick = new SwerveJoysticks(controller, swerve);
  }

  @Override
  public void teleopPeriodic() {
    // moduleLeftFront.setVector(new Vector(2, 90, Representation.Polar));
    // moduleLeftRear.setVector(new Vector(2, 90, Representation.Polar));
    // moduleRightFront.setVector(new Vector(2, 90, Representation.Polar));
    // moduleRightRear.setVector(new Vector(2, 0, Represention.Polar));
    // swerve.setRelativeSwerveVelocoties(new Vector(0, 0, Representation.Polar), 6);
    // joystick.execute();
    swerve.setRelativeSwerveVelocoties(new Vector(0, 0, Representation.Polar), 2);

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
    // CommandScheduler.getInstance().schedule(new FunctionalCommand(() -> {}, () -> {System.out.println("@");}, (Boolean bool) -> {}, () -> {return false;}));

    m_robotContainer.setSocket();
    
    LogCommand log = new LogCommand(1, () -> {return drivingRightFront.getRpm();}, m_robotContainer.outputStream);
    LogCommand log2 = new LogCommand(21, () -> {return rotationLeftFront.getAngle();}, m_robotContainer.outputStream);
    
    CommandScheduler.getInstance().schedule(log);
    log2.schedule();

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
