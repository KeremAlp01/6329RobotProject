// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Vision", inputs);
    io.updateInputs(inputs);
  }

  public double getTY() {
    return io.getTY();
  }

  public void setReferencePose(Pose2d pose) {
    io.setReferencePose(pose);
  }

  public void updateInputs() {}
}
