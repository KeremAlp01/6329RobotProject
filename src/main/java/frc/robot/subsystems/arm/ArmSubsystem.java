// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward ff;

  /** Creates a new armSubsystem. */
  public enum armState {
    HOME,
    AMPPOSE,
    SPEAKER
  }

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  public void stopArm() {
    io.stopArm();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setArmAngle2(double targetAngle2, double ff) {
    io.setArmAngle2(
        targetAngle2, ArmConstants.ff.calculate(targetAngle2, ArmConstants.velocitySetpoint));
  }

  public void setArmAngle(double targetAngle) {
    io.setArmAngle(targetAngle);
  }

  public double getPosition() {
    return inputs.armPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SmartDashboard.putNumber("Arm Angle ", getPosition());
    SmartDashboard.putNumber("Arm Voltage", inputs.armVolts);

    Logger.processInputs("Arm", inputs);

    // This method will be called once per scheduler run
  }
}
