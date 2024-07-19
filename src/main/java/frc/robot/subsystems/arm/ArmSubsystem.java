// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  /** Creates a new armSubsystem. */

  public enum armState {
    HOME,
    AMPPOSE,
    SPEAKER
  }
  
  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  public void stopArm(){
    io.stopArm();
  }

  public void setArmVolts(double voltage){
    io.setArmVolts(voltage);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // This method will be called once per scheduler run
  }
}
