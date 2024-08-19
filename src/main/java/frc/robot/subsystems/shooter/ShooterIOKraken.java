// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterIOKraken implements ShooterIO {
  private final TalonFX shooter1 = new TalonFX(0); // left
  private final TalonFX shooter2 = new TalonFX(1); // right

  private final StatusSignal<Double> leftRPM = shooter1.getVelocity();
  private final StatusSignal<Double> rightRPM = shooter2.getVelocity();
  private final StatusSignal<Double> leftVoltage = shooter1.getMotorVoltage();
  private final StatusSignal<Double> rightVoltage = shooter2.getMotorVoltage();

  private double leftTargetRPM = 0.0;
  private double rightTargetRPM = 0.0;

  public ShooterIOKraken() {
    configShooterTalonFX(shooter1);
    configShooterTalonFX(shooter2);
    shooter1.setInverted(false);
    shooter2.setInverted(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftRPM, rightRPM, leftVoltage, rightVoltage);
  }

  public void configShooterTalonFX(TalonFX talon) {
    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = false;

    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot1.kS = 0;
    config.Slot1.kV = 0;
    config.Slot1.kA = 0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftRPM, rightRPM, leftVoltage, rightVoltage);

    inputs.leftRPM = shooter1.getVelocity().getValueAsDouble() * 60;
    inputs.rightRPM = shooter1.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setVoltage(double voltage) {
    shooter1.setVoltage(voltage);
    shooter2.setVoltage(voltage);
  }

  @Override
  public double getLeftRPM() {
    return shooter2.getVelocity().getValueAsDouble();
  }

  @Override
  public double getRightRPM() {
    return shooter1.getVelocity().getValueAsDouble();
  }

  @Override
  public void setTargetRPM(double leftRPM, double rightRPM, double ffVolts) {
    leftTargetRPM = leftRPM;
    rightTargetRPM = rightRPM;
    shooter2.setControl(new VelocityVoltage(leftRPM / 60).withSlot(0));
    shooter1.setControl(new VelocityVoltage(rightRPM / 60).withSlot(0));
  }

  @Override
  public double getLeftTargetRPM() {
    return leftTargetRPM;
  }

  @Override
  public double getRightTargetRPM() {
    return rightTargetRPM;
  }
}
