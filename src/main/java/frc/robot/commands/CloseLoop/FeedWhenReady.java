// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class FeedWhenReady extends Command {
  private final Shooter mShooter;
  private final ArmSubsystem mArm;
  private final FeederSubsystem mFeeder;
  private final Drive mDrive;
  /** Creates a new FeedWhenRead. */
  public FeedWhenReady(Shooter mShooter, ArmSubsystem mArm, FeederSubsystem mFeeder, Drive mDrive) {
    this.mShooter = mShooter;
    this.mArm = mArm;
    this.mFeeder = mFeeder;
    this.mDrive = mDrive;

    addRequirements(mFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isHeadingTarget = mDrive.fieldCentricHeadingLockRequest.HeadingController.atSetpoint();

    if (Constants.kShootingParams.isShooterAtSetpoint(
            mShooter.getLeftRPM(), mShooter.getLeftTargetRPM())
        & Constants.kShootingParams.isShooterAtSetpoint(
            mShooter.getRightRPM(), mShooter.getRightTargetRPM())
        & Constants.kShootingParams.isShooterPivotAtSetpoint(
            mArm.getArmAngle(), mArm.getTargetAngle())
        & mFeeder.getFeederSensorValue()
        & isHeadingTarget == true) {

      mFeeder.setVoltage(10);

    } else {

      mFeeder.setVoltage(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
