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

    public ControllerCalculator(PS4Controller controller) {
        this.controller = controller;
    }

    public Vector getLocationVector() {
        Vector vector = new Vector(
            this.getX(),
            this.getY(),
            Representation.Cartisian
        );
        vector.changeMagnitude(2);
        return vector;
    }

    public double getX() {
        double value = this.controller.getLeftX();
        if (value < 0.15) {
            value = 0;
        }
        return value;
    }

    public double getY() {
        double value = -this.controller.getLeftY();
        if (value < 0.15) {
            value = 0;
        }
        return value;
    }

    public double getOmega() {
        return this.controller.getRightX();
    }
}
