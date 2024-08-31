// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void updateInputs() {}
}
