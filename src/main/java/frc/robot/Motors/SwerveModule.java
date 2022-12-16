// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors;
import frc.robot.Motors.abstractMotors.MotorInterface;
import frc.robot.Motors.abstractMotors.RotationMotorInterface;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

/** Add your docs here. */
public class SwerveModule {
    private MotorInterface movementMotor;
    private RotationMotorInterface rotationMotor;

    double RotationToMetersRatio; // how many rotations are one meter, rotation / 1 meter;

    public SwerveModule(MotorInterface movementMotor, RotationMotorInterface rotationMotor, double wheelRadius) {
        this.movementMotor = movementMotor;
        this.rotationMotor = rotationMotor;
        
        this.RotationToMetersRatio = 1 / (2 * Math.PI * wheelRadius) * 60;
    }
    
    public double getAngle(){
        return rotationMotor.getAngleByCanCoder();
    }

    public void setAngle(double angle){
        rotationMotor.setAngle(angle);
    }

    public void setVelocity(double metersPerSecond){
        movementMotor.setRpm(metersPerSecond * RotationToMetersRatio);
    }

    public double getVelocity(){
        // in meters / sec
        return movementMotor.getRpm() / RotationToMetersRatio;
    }

    public void setVector(Vector vector){
        this.setVelocity(vector.getMagnitude());
        this.setAngle(vector.getAngle());
    }

    public Vector getVector() {
        return new Vector(
            this.getVelocity(),
            this.getAngle(),
            Representation.Polar
        );
    }
}
