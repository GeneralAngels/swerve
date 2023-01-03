// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Motors.abstractMotors.MotorInterface;
import frc.robot.Motors.abstractMotors.RotationMotorInterface;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

/** Add your docs here. */
public class SwerveModule {
    private MotorInterface movementMotor;
    private RotationMotorInterface rotationMotor;

    double direction = 1;

    double RotationToMetersRatio; // how many rotations are one meter, rotation / 1 meter;

    public SwerveModule(MotorInterface movementMotor, RotationMotorInterface rotationMotor, double wheelRadius) {
        rotationMotor.setEncoder();
        this.movementMotor = movementMotor;
        this.rotationMotor = rotationMotor;
        
        this.RotationToMetersRatio = 1 / (2 * Math.PI * wheelRadius) * 60;
    }
    
    public double getAngle(){
        return rotationMotor.getAngleByFalcon();
    }

    public void setAngle(double angle){
        rotationMotor.setAngle(angle);
    }
    
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static double[] optimize(double desiredAngle, double desiredVelocity, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredAngle);
        double targetSpeed = desiredVelocity;
        double delta = targetAngle - currentAngle;
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return new double[] {targetAngle, targetSpeed};
    }

    public void setVelocity(double metersPerSecond){
        movementMotor.setRpm(metersPerSecond * RotationToMetersRatio);
    }

    public double getVelocity(){
        // in meters / sec
        return movementMotor.getRpm() / RotationToMetersRatio;
    }

    public void setVector(Vector vector){
        double[] optimizedStates = optimize(vector.getAngle(), vector.getMagnitude(), getAngle());
        
        this.setVelocity(optimizedStates[1]);
        this.setAngle(optimizedStates[0]);
    }

    public void setState(SwerveModuleState desiredState) {
        double[] optimizedStates = optimize(desiredState.angle.getDegrees(), desiredState.speedMetersPerSecond, getAngle());

        this.setVelocity(optimizedStates[1]);
        this.setAngle(optimizedStates[0]);
    }

    public Vector getVector() {
        return new Vector(
            this.getVelocity(),
            this.getAngle(),
            Representation.Polar
        );
    }
}
