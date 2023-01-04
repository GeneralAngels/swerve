// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.falcon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

/** Add your docs here. */
public class RotationCancoderFalcon {
    TalonFX falcon;
    int canCoderPort;
    double homeAngle;

    double drivingToDrivenGearRatio = 150 / 7;
    double tick_0;

    CANCoder canCoder;
    
    public RotationCancoderFalcon(int canCoderPort, TalonFX falcon, double homeAngle) {
        this.falcon = falcon;
        this.canCoderPort = canCoderPort;
        this.homeAngle = homeAngle;
        canCoder = new CANCoder(canCoderPort);
        configFalconForCanCoder();
    }
    
    public void configFalconForCanCoder() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.remoteFilter0 = new FilterConfiguration();
        config.remoteFilter0.remoteSensorDeviceID = canCoderPort;
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

        falcon.configAllSettings(config);
        falcon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    }

    public double getAngle() {
        return this.falcon.getSelectedSensorPosition(0);
    }

    public void setEncoder() {
        double a = this.anglesToTicks(this.homeAngle - this.canCoder.getAbsolutePosition());
        double y = this.falcon.getSelectedSensorPosition(0);

        tick_0 = y - a;
    }

    public double anglesToTicks(double angles) {
        return angles * 2048 / 360 * drivingToDrivenGearRatio;
    }
    
    public double ticksToAngle(double ticks){
        return ((ticks / 2048) * 360 / drivingToDrivenGearRatio);
    }
}
