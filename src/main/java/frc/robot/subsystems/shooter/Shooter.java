// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // State
  public enum ShooterState {
    IDLE,
    SPINNING_UP,
    TARGETRPM
  }

  public static ShooterState shooterState = ShooterState.IDLE;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setVoltage(double rightVoltage, double leftVoltage) {
    io.setVoltage(rightVoltage, leftVoltage);
  }

  public void setTargetRPM(double leftRPM, double rightRPM) {
    io.setTargetRPM(leftRPM, rightRPM);
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putString("ShooterState", getShooterState().toString());

    // This method will be called once per scheduler run
  }
}
