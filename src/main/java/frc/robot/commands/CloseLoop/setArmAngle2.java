// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class setArmAngle2 extends Command {
  private final ArmSubsystem arm;
  private final double targetAngle;
  private final double feedforward;

  /** Creates a new setArmAngle. */
  public setArmAngle2(ArmSubsystem arm, double targetAngle, double feedforward) {
    this.arm = arm;
    this.targetAngle = targetAngle;
    this.feedforward = feedforward;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.setArmAngle(targetAngle, feedforward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
