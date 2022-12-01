// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors;
import frc.robot.Motors.abstractMotors.AbstractMotor;
import frc.robot.Motors.falcon.RotationFalcon;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

/** Add your docs here. */
public class SwerveModule {
    private AbstractMotor movementMotor;
    private RotationFalcon rotationMotor;

    double rotationRatio; // how many rotations are one meter, rotation / 1 meter;

    public SwerveModule(AbstractMotor movementMotor, RotationFalcon rotationMotor, double rotationRatio) {
        this.movementMotor = movementMotor;
        this.rotationMotor = rotationMotor;
        
        this.rotationRatio = rotationRatio;
    }
    
    public double getAngle(){
        return rotationMotor.getAngleByCanCoder();
    }

    public void setAngle(double angle){
        rotationMotor.setAngle(angle);
    }

    public void setVelocity(double metersPerSecond){
        movementMotor.setRpm(metersPerSecond * rotationRatio);
    }

    public double getVelocity(){
        // in meters / sec
        return movementMotor.getRpm() / rotationRatio;
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
