// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Motors.falcon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Motors.abstractMotors.RotationMotorInterface;

/** Add your docs here. */
public class RotationFalcon extends Falcon implements RotationMotorInterface {
    double motorOffset;
    double homeAngle;

    int CANCoderPort;
    double direction = 1;

    double offsetConstant;

    public CANCoder canCoder;

    double tick_0;

    public RotationFalcon
    (
        TalonFX talon,
        int CANCoderPort,
        int kPIDLoopIdx,
        double peakOutputForward, double peakOutputReverse,
        double Kf, double Kp, double Ki, double Kd,
        double motorOffset, double homeAngle,
        Boolean inverted, Boolean invertSensorPhase
    ) 
    {
        super(talon, kPIDLoopIdx, peakOutputForward, peakOutputReverse, Kf, Kp, Ki, Kd, 1);

        this.homeAngle = homeAngle;
        this.motorOffset = motorOffset;
        
        this.motorOffset = motorOffset;
        this.CANCoderPort = CANCoderPort;
        this.canCoder = new CANCoder(CANCoderPort);

        this.config(_talon, kPIDLoopIdx, peakOutputForward, peakOutputReverse, Kf, Kp, Ki, Kd, invertSensorPhase);
        // this._talon.setInverted(inverted);
        this._talon.setSensorPhase(true);
    }

    @Override
    public void config
    (
        TalonFX _talon, int kPIDLoopSlotIdx,
        double peakOutputForward, double peakOutputReverse,
        double Kf, double Kp, double Ki, double Kd, 
        Boolean invertSensorPhase
    )
    {
        System.out.println("configing correctly");        
        // this._talon.configRemoteFeedbackFilter(this.CANCoderPort, RemoteSensorSource.CANCoder, 0);
        // this._talon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        _talon.configFactoryDefault();

        this._talon.setSensorPhase(invertSensorPhase);
        this._talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        this._talon.config_kF(kPIDLoopSlotIdx, Kf, kTimeoutMs);
		this._talon.config_kP(kPIDLoopSlotIdx, Kp, kTimeoutMs);
		this._talon.config_kI(kPIDLoopSlotIdx, Ki   , kTimeoutMs);
		this._talon.config_kD(kPIDLoopSlotIdx, Kd, kTimeoutMs);
        // _talon.configAllowableClosedloopError(0, kPIDLoopSlotIdx, kTimeoutMs);
        this._talon.selectProfileSlot(kPIDLoopSlotIdx, 0); // pidIdx = 0 because we use regular closed loop controller
        // this._talon.configFeedbackNotContinuous(true, kTimeoutMs);
    }

    public void setAngle(double angle) {
        double wantedTick = this.anglesToTicks(angle) + tick_0;
        double curretPosition = this.getPosition();

        double error = wantedTick - curretPosition;
        double finalTarget;

        if (error > this.anglesToTicks(180)) {
            finalTarget = wantedTick - this.anglesToTicks(360);
        }
        else if (error < this.anglesToTicks(-180)) {
            finalTarget = wantedTick + this.anglesToTicks(360);
        }
        else {
            finalTarget = wantedTick;
        }
                
        this.setPosition(finalTarget);
    }

    public void setFalconEncoder() {
        double a = this.anglesToTicks(this.homeAngle - this.canCoder.getAbsolutePosition());
        double y = this._talon.getSelectedSensorPosition(0);

        tick_0 = y - a;
    }

    public double anglesToTicks(double angles) {
        return angles * 2048 * 150 / (360 * 7);
    }
    
    public double ticksToAngle(double ticks){
        return ((ticks / 2048) / 150) * (360 * 7);
    }

    public double getAngleByCanCoder(){
        // by canCoder
        return this.canCoder.getAbsolutePosition() - this.homeAngle;
    }

    public double getAngleByFalcon() {
        return ticksToAngle(this._talon.getSelectedSensorPosition(0) - tick_0);
    }
    
    @Override
    public void setRpm(double rpm) {
        super.setRpm(rpm * direction);
    }
}
