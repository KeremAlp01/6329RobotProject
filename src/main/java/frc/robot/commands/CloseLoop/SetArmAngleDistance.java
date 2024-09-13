// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;

public class SetArmAngleDistance extends Command {
  private final ArmSubsystem arm;
  private final Drive drive;
  private double targetAngle;
  /** Creates a new SetArmAngleDistance. */
  public SetArmAngleDistance(ArmSubsystem arm, Drive drive) {
    this.arm = arm;
    this.drive = drive;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = drive.getDistanceToSpeaker();
    targetAngle =
        Constants.kPivotMap.getInterpolated(new InterpolatingDouble(Double.valueOf(distance)))
            .value;

    arm.setArmAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getPosition() - targetAngle) <= 0.5;
  }
}
