// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Add your docs here. */
public class Vector {
    public enum Representation{
        Cartisian,
        Polar
    }
    
    double x; 
    double y;
    double magnitude;
    double degreesAngle;
    double raidansAngle;
    Representation representation;
    
    public Vector(double x, double y, Representation rep){
        /*
        * x y for Cartizian and magnitude and angle for 
        */
        this.representation = rep;
        if(rep == Representation.Cartisian)
        {
            this.x = x;
            this.y = y;
            this.raidansAngle = Math.atan2(y, x);
            this.degreesAngle = Math.toDegrees(raidansAngle);
            this.magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }
        else
        {
            this.magnitude = x;
            this.degreesAngle = y;
            this.raidansAngle = Math.toRadians(degreesAngle);
            this.x = Math.cos(this.raidansAngle) * this.magnitude; // meter
            this.y = Math.sin(this.raidansAngle) * this.magnitude; // meter
        }
    }
    public double getMagnitude() {
        return this.magnitude;
    }

    public double getAngle() {
        return this.degreesAngle;
    }
    public void changeMagnitude(double magnitude) {
        this.magnitude = magnitude;
        this.x = Math.cos(this.raidansAngle) * magnitude;
        this.y = Math.sin(this.raidansAngle) * magnitude;
    }

    public void rotateVector(double rotationRadiansAngle) {
        this.raidansAngle -= rotationRadiansAngle;
        this.degreesAngle = Math.toDegrees(this.raidansAngle);

        this.x = Math.cos(this.raidansAngle) * this.magnitude;
        this.y = Math.sin(this.raidansAngle) * this.magnitude;
    }

    public double getX() {
        return this.x;
    }
    
    public double getY() {
        return this.y;
    }
}
