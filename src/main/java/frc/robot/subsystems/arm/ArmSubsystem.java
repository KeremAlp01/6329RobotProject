// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward ff;

  private final Translation3d pivotPosition = new Translation3d(0, 0.05, 0.63);

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();

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

  public Rotation3d getArmAngle() {
    return io.getArmAngle();
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

    publisher.set(new Pose3d(pivotPosition, io.getArmAngle()));
    // This method will be called once per scheduler run
  }
}
