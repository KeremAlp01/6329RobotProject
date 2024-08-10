// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.lib.Conversions;

/** Add your docs here. */
public class ArmIOFalcon implements ArmIO {
  // Motor
  private final TalonFX armMotor = new TalonFX(22, "Canbus");

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(DCMotor.getKrakenX60(1), 1, 0.1, 0.5, -1, 200, false, 0);
  private final PIDController pid = new PIDController(0.1, 0, 0);
  // Status signals
  private final StatusSignal<Double> armPosition = armMotor.getPosition();
  private final StatusSignal<Double> armVolts = armMotor.getMotorVoltage();
  //

  private double armVoltage;


  public ArmIOFalcon() {
    configArmTalonFX(armMotor);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, armPosition, armVolts);
  }

  public void configArmTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = 300;
    config.MotionMagic.MotionMagicAcceleration = 300;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 180;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -180;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(armPosition, armVolts);
    inputs.armVolts = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armPosition = armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setArmAngle(double targetAngle) {
    armMotor.setControl(
        new MotionMagicVoltage(Conversions.degreesToRotations(targetAngle, ArmConstants.kGearRatio))
            .withSlot(0));
  }


  @Override
  public void setVoltage(double votlage) {
    armMotor.setVoltage(votlage);
  }

  @Override
  public void stopArm() {
    armMotor.stopMotor();
  }
}
