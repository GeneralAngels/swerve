// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.DataOutputStream;
import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LogCommand extends CommandBase {
  int motorId;
  Callable<Double> getValueFunction;
  DataOutputStream outputStream;

  /** Creates a new LogCommand. */
  public LogCommand(int motorId, Callable<Double> getValueFunction, DataOutputStream outputStream) {
    this.motorId = motorId;
    this.getValueFunction = getValueFunction;
    this.outputStream = outputStream;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    try {
      this.outputStream.writeInt(motorId);
      System.out.println(motorId + ", " + this.getValueFunction.call());
      this.outputStream.writeDouble(this.getValueFunction.call());
      this.outputStream.flush();
    }

    catch (Exception exception) {
      // exception.printStackTrace();
    }

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
