// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ArmIOFalcon implements ArmIO{
    //Motor
    private final TalonFX armMotor = new TalonFX(22, "Canbus");

    //Status signals
    private final StatusSignal<Double> armAngle = armMotor.getPosition();
    private final StatusSignal<Double> armVolts = armMotor.getMotorVoltage();



    public ArmIOFalcon(){
     configArmTalonFX(armMotor);
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, armAngle,armVolts);
    }


    public void configArmTalonFX(TalonFX talon){

        talon.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration(); 

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0.0;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = 1.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicAcceleration = 0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        BaseStatusSignal.refreshAll(armAngle,armVolts);
        inputs.armVolts = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.armAngle = armMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setArmVolts(double votlage){
        armMotor.setVoltage(votlage);
    }

    @Override
    public void stopArm(){
        armMotor.stopMotor();
    }
}
