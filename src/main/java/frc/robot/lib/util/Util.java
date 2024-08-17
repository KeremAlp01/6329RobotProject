// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class Util {
  public static final double kEpsilon = 1e-12;

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(
      Rotation3d rotation3d, Rotation3d rotation3d2, double mShooterAllowableErrorRPM) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'epsilonEquals'");
  }
}
