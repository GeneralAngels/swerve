// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motors.SwerveModule;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

public class SwerveDriveTrain extends SubsystemBase {
  SwerveModule rightFront;
  SwerveModule rightRear;
  SwerveModule leftRear;
  SwerveModule leftFront;

  WPI_PigeonIMU gyro;

  double width; 
  double length;

  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain(
    SwerveModule rightFront, SwerveModule rightRear, SwerveModule leftRear, SwerveModule leftFront,
    WPI_PigeonIMU gyro, 
    double width, double length) {
    
    
    this.rightFront = rightFront;
    this.rightRear = rightRear;
    this.leftRear = leftRear;
    this.leftFront = leftFront;

    this.gyro = gyro;
    this.gyro.calibrate();

    this.width = width;
    this.length = length;
  }

  public void setRelativeSwerveVelocoties(Vector vector, double omega) {
    // Module 1:
    Vector module_1_vector = new Vector(
      (vector.getX() - omega * (length / 2)), 
      (vector.getY() + omega * (width / 2)), 
      Representation.Cartisian
    );
    this.rightFront.setVector(module_1_vector);

    // Module 2:
    Vector module_2_vector = new Vector(
      (vector.getX() - omega * (length / 2)), 
      (vector.getY() - omega * (width / 2)), Representation.Cartisian
    );
    this.rightRear.setVector(module_2_vector);

    // Module 3:
    Vector module_3_vector = new Vector(
      (vector.getX() - omega * (length / 2)), 
      (vector.getY() + omega * (width / 2)), 
      Representation.Cartisian
    );
    this.leftRear.setVector(module_3_vector);

    // Module 4:
    Vector module_4_vector = new Vector(
      (vector.getX() + omega * (length / 2)), 
      (vector.getY() + omega * (width / 2)), 
      Representation.Cartisian
    );
    this.leftFront.setVector(module_4_vector);
  }

  public void setAbsoluteSwerveVelocoties(Vector absoluteVector, double omega) {
    this.setRelativeSwerveVelocoties(
      SwerveDriveTrain.toRelativeVector(absoluteVector, this.gyro.getAngle()), omega
    );
  }
  
  public static Vector toRelativeVector(Vector absoluteVector, double robotAngle) {
    double robotRadiansAngle = Math.toRadians(robotAngle);
    
    return new Vector(
      Math.cos(robotRadiansAngle) * absoluteVector.getX() - Math.sin(robotRadiansAngle) * absoluteVector.getY(),
      Math.sin(robotRadiansAngle) * absoluteVector.getX() + Math.cos(robotRadiansAngle) * absoluteVector.getY(),
      Representation.Cartisian
    );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
