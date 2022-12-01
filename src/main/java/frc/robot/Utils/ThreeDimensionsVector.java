// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Add your docs here. */
public class ThreeDimensionsVector extends Vector {
    double omega;

    public ThreeDimensionsVector(double x, double y, double omega, Representation representation) {
        super(x, y, representation);
        this.omega = omega;
    }

    public double getOmega() {
        return this.omega;
    }
}
