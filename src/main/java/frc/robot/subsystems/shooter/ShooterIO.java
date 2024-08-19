// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double rightVoltage;
    public double leftVoltage;
    public double leftRPM;
    public double rightRPM;
  }

  public default void setVoltage(double voltage) {}

  public default void setTargetRPM(double leftRPM, double rightRPM, double ffVolts) {}

  public default double getLeftRPM() {
    return 0;
  }

  public default double getRightRPM() {
    return 0;
  }

  public default double getLeftTargetRPM() {
    return 0;
  }

  public default double getRightTargetRPM() {
    return 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
