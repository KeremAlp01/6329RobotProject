// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class SetShooterRPM extends Command {
  private final Shooter mShooter;
  private final double rightRPM;
  private final double leftRPM;
  private final boolean shouldStop;
  
  /** Creates a new setAShooterRPm. */
  public SetShooterRPM(Shooter mShooter, double leftRPM, double rightRPM, boolean shouldStop) {
    this.mShooter = mShooter;
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;
    this.shouldStop = shouldStop;
    addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setShooterState(ShooterState.SPINNING_UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setTargetRPM(leftRPM, rightRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (shouldStop) {
      mShooter.setVoltage(0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    mShooter.setShooterState(ShooterState.IDLE);
    return false;
  }
}
