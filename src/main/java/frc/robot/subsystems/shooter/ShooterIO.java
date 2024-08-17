// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
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

  public default void setVoltage(double rightVotlage, double leftVoltage) {}

  public default void setTargetRPM(double leftRPM, double rightRPM) {}

  public default double getLeftRPM() {
    return 0;
  }

  public default double getRightRPM() {
    return 0;
  }

  public default double getLeftTargetRPM(){
    return Double.NaN;
  }

  public default double getRightTargetRPM(){
    return Double.NaN;
  }
  
  public default void updateInputs(ShooterIOInputs inputs) {}
}
