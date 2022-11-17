// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Vector;
import frc.robot.subsystems.SwerveDriveTrain;

public class SwerveJoysticks extends CommandBase {
  SwerveDriveTrain swerve;
  ControllerCalculator controller;
  
  /** Creates a new SwerveJoysticks. */
  public SwerveJoysticks(PS4Controller controller, SwerveDriveTrain swerve) {
    addRequirements(swerve);
    
    this.swerve = swerve;
    this.controller = new ControllerCalculator(controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Vector locationVector = this.controller.getLocationVector();
    // System.out.println(locationVector.getAngle() + ", " + locationVector.getMagnitude());
    this.swerve.setSwerveVelocities(locationVector, this.controller.getOmega());
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
