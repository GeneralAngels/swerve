// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.DataOutputStream;
import java.util.concurrent.Callable;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LogCommand extends CommandBase {
  int motorId;
  Callable<Double> getValueFunction;
  DataOutputStream outputStream;
  boolean printed = false;
  StopWatch stopWatch = new StopWatch();
  double lastTimeSent = 0;

  /** Creates a new LogCommand. */
  public LogCommand(int motorId, Callable<Double> getValueFunction, DataOutputStream outputStream) {
    this.motorId = motorId;
    this.getValueFunction = getValueFunction;
    this.outputStream = outputStream;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    try {
      if (stopWatch.getDuration() - lastTimeSent >= 0.05) {
        this.outputStream.writeInt(motorId);
        this.outputStream.writeDouble(this.stopWatch.getDuration());
        this.outputStream.writeDouble(this.getValueFunction.call());
        this.outputStream.flush();
        System.out.println(motorId + ", " + this.getValueFunction.call());
        lastTimeSent = stopWatch.getDuration();
      }
      
    }

    catch (Exception exception) {
      if (printed == false) {
        exception.printStackTrace();
        printed = true;
      }
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
