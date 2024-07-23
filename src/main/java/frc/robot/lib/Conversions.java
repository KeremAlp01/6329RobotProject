// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class Conversions {
  public static double degreesToRotations(double degrees, double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }
}
