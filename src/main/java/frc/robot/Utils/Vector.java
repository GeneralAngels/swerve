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
    double angle;
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
            this.angle = Math.toDegrees(raidansAngle);
            this.magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }
        else
        {
            this.magnitude = x;
            this.angle = y;
            this.raidansAngle = Math.toRadians(angle);
            this.x = Math.cos(this.raidansAngle) * this.magnitude;
            this.y = Math.sin(this.raidansAngle) * this.magnitude;
        }
    }
    public double getMagnitude() {
        return this.magnitude;
    }

    public double getAngle() {
        return this.angle;
    }
    public void changeMagnitude(double magnitude) {
        this.x = Math.cos(this.raidansAngle) * magnitude;
        this.y = Math.sin(this.raidansAngle) * magnitude;
    }

    public double getX() {
        return this.x;
    }
    
    public double getY() {
        return this.y;
    }
}
