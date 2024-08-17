// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.ShootingParameters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class AutoConstants {
    public static final double kPThetaController = 0;
    public static final double kPXController = 0;
  }

  public static class ArmConstants {
    public static final double kGearRatio = 1;
    public static final double kS = 0.1;
    public static final double kA = 0.1;
    public static final double kV = 0.1;
    public static final double kG = 0.1;
    public static final double velocitySetpoint = 0;
    public static final ArmFeedforward ff =
        new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  }

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static double[][] kRPMValues = {
    {6.7, 1084},
    {5.5, 990},
    {4.5, 900},
    {3.5, 813},
    {2.5, 770},
    {2, 730},
    {1.5, 740},
  };

  public static double[][] kPivotValues = {
    {6.7, 25},
    {5.5, 21.66},
    {4.5, 19.2},
    {3.5, 12},
    {2.5, 7},
    {2, 4.5},
    {1.5, 0}
  };

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotMap =
      new InterpolatingTreeMap<>();
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap =
      new InterpolatingTreeMap<>();

  static {
    for (double[] pair : kRPMValues) {
      kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    for (double[] pair : kPivotValues) {
      kPivotMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
  }

  public static final ShootingParameters kShootingParams =
      new ShootingParameters(kPivotMap, kRPMMap, 300, 0.5);
}
