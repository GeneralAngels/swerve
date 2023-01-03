// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Utils.Vector;
import frc.robot.Utils.Vector.Representation;

/** Add your docs here. */
public class ControllerCalculator {
    PS4Controller controller;          

    double maxSpeed = 1.75;
    double maxOmega = 3;

    public ControllerCalculator(PS4Controller controller) {
        this.controller = controller;
    }

    public Vector getLocationVector() {
        Vector vector = new Vector(
            this.getX(),
            -this.getY(),
            Representation.Cartisian
        );
        vector.changeMagnitude(this.getGas() * maxSpeed);
        return vector;
    }

    public double getGas() {
        return (this.controller.getR2Axis() + 1) / 2;
    }

    public double getX() {
        double value = this.controller.getLeftX();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value;
    }

    public double getY() {
        double value = this.controller.getLeftY();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value;
    }

    public double getOmega() {
        double value = -this.controller.getRightX();
        if (Math.abs(value) < 0.05) {
            value = 0;
        }
        return value * maxOmega;
    }
}
