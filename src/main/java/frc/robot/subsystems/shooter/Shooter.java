// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.00);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0016);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    ;
  }

  public void setTargetRPM(double leftRPM, double rightRPM) {
    io.setTargetRPM(leftRPM, rightRPM, ffModel.calculate(leftRPM));
  }

  public void setShooterState(ShooterState state) {
    shooterState = state;
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  public double getLeftRPM() {
    return io.getLeftRPM();
  }

  public double getRightRPM() {
    return io.getRightRPM();
  }

  public double getLeftTargetRPM() {
    return io.getLeftTargetRPM();
  }

  public double getRightTargetRPM() {
    return io.getRightTargetRPM();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putString("ShooterState", getShooterState().toString());

    // This method will be called once per scheduler run
  }
}
