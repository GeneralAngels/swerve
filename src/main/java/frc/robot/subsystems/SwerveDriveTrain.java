// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motors.SwerveModule;
import frc.robot.Utils.Vector3d;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

public class SwerveDriveTrain extends SubsystemBase {
  SwerveModule rightFront;
  SwerveModule rightRear;
  SwerveModule leftRear;
  SwerveModule leftFront;

  public WPI_PigeonIMU gyro;

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
    this.gyro.reset();
    System.out.println("after callibration");

    this.width = width;
    this.length = length;
  }

  public void setRelativeSwerveVelocoties(Vector vector, double omega) {    
    // Module 1:
    Vector rightFrontVector = new Vector(
      (vector.getX() - omega * (length / 2)), 
      (vector.getY() + omega * (width / 2)), 
      Representation.Cartisian
    );


    // System.out.println(String.format("rightFront vector: angle = %f, velocity = %f", vector.getAngle(), vector.getMagnitude()));
    this.rightFront.setVector(rightFrontVector);

    // Module 2:
    Vector rightRearVector = new Vector(
      (vector.getX() - omega * (length / 2)), 
      (vector.getY() - omega * (width / 2)), Representation.Cartisian
    );
    this.rightRear.setVector(rightRearVector);

    // Module 3:
    Vector leftFrearVector = new Vector(
      (vector.getX() + omega * (length / 2)), 
      (vector.getY() - omega * (width / 2)), 
      Representation.Cartisian
    );
    this.leftRear.setVector(leftFrearVector);

    // Module 4:
    Vector leftFrontVector = new Vector(
      (vector.getX() + omega * (length / 2)), 
      (vector.getY() + omega * (width / 2)), 
      Representation.Cartisian
    );
    this.leftFront.setVector(leftFrontVector);
  }

  public void setAbsoluteSwerveVelocoties(Vector absoluteVector, double omega) {
    this.setRelativeSwerveVelocoties(
      SwerveDriveTrain.toRelativeVector(absoluteVector, -this.gyro.getAngle()), omega
    );
  }
  
  public static Vector toRelativeVector(Vector absoluteVector, double robotAngle) {
    
    double robotRadiansAngle = Math.toRadians(robotAngle);
    // Rotation Matrix
    return new Vector(
      Math.cos(robotRadiansAngle) * absoluteVector.getX() - Math.sin(robotRadiansAngle) * absoluteVector.getY(),
      Math.sin(robotRadiansAngle) * absoluteVector.getX() + Math.cos(robotRadiansAngle) * absoluteVector.getY(),
      Representation.Cartisian
    );
  }

  public Vector3d getRobotVector() {
    Vector rightFrontVector = rightFront.getVector();
    Vector rightRearVector = rightRear.getVector();
    Vector leftRearVector = leftRear.getVector();
    Vector leftFrontVector = leftFront.getVector();

    double xVelocity = (rightFrontVector.getX() + rightRearVector.getX() + leftRearVector.getX() + leftFrontVector.getX()) / 4;
    double yVelocity = (rightFrontVector.getY() + rightRearVector.getY() + leftRearVector.getY() + leftFrontVector.getY()) / 4;

    Vector3d vector = new Vector3d(
      xVelocity,
      yVelocity,
      (rightFrontVector.getX() - xVelocity) / -(length / 2),
      Representation.Cartisian
    );

    vector.rotateVector(Math.toRadians(90));
    return vector;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
