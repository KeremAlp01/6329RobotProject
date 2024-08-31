// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.util.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private final double xyStdDevCoefficient = 0.3;
  private static final String limelight_name = new String("limelight");
  private static double targetX;
  private static double targetY;
  private static boolean isValid = false;

  public VisionIOLimelight() {
    SmartDashboard.putNumber("Limelight_TY", targetY);
  }

  @Override
  public void updateInputs() {
    isValid = LimelightHelpers.getTV("limelight");
    targetX = LimelightHelpers.getTX("limelight");
    targetY = LimelightHelpers.getTY("limelight");

    LimelightHelpers.setPriorityTagID("limelight", Robot.getAlliance() == Alliance.Red ? 4 : 8);

    final LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight_name);

    if (LimelightHelpers.getTV(limelight_name)) {

      final Pose2d pose = limelightMeasurement.pose;

      final double timestamp = limelightMeasurement.timestampSeconds;
      final double avgDist = limelightMeasurement.avgTagDist;
      final int tagCount = limelightMeasurement.tagCount;
      double xyStdDev = Double.POSITIVE_INFINITY;
      if (avgDist > 0 && tagCount > 0) {
        xyStdDev = xyStdDevCoefficient * Math.pow(avgDist, 2.5) / tagCount;
      }

      SmartDashboard.putNumberArray(
          "visionpos" + limelight_name,
          new double[] {pose.getX(), pose.getY(), new Rotation2d().getDegrees()});

      if (tagCount > 0
          && avgDist > 0
          && !Double.isNaN(pose.getX())
          && !Double.isNaN(pose.getY())
          && !Double.isInfinite(pose.getX())
          && !Double.isInfinite(pose.getY())) {

        if (pose.getX() > 0.0
            && pose.getX() <= 16.58
            && pose.getY() <= 8.1
            && pose.getY() > 0.0
            && RobotContainer.drive.getCurrentChassisSpeeds().omegaRadiansPerSecond
                <= Math.PI / 2.0) {
          if (tagCount < 2) {

            if (avgDist <= 1.5) {
              RobotContainer.drive.addVisionMeasurement(
                  pose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY));
            }

          } else {
            if (avgDist <= 3.0) {
              RobotContainer.drive.addVisionMeasurement(
                  pose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY));
            }
          }
        }
      }
    }
  }

  @AutoLogOutput(key = "Limelight/TY")
  public void getTY() {
    LimelightHelpers.getTY(limelight_name);
  }

  public void getTX() {
    LimelightHelpers.getTX(limelight_name);
  }

  public void getDistance() {}
}
