package frc.robot.lib.util;

public class ShootingParameters {
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterPivotMap;
  private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterRPMMap;
  private final double mShooterAllowableErrorRPM; // rpm
  private final double mPivotAllowableErrorDegrees; // °

  public ShootingParameters(
      InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterPivotMap,
      InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterRPMMap,
      double shooterAllowableErrorRPM,
      double pivotAllowableErrorDegrees) {
    this.mShooterPivotMap = shooterPivotMap;
    this.mShooterRPMMap = shooterRPMMap;
    this.mShooterAllowableErrorRPM = shooterAllowableErrorRPM;
    this.mPivotAllowableErrorDegrees = pivotAllowableErrorDegrees;
  }

  public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterPivotMap() {
    return mShooterPivotMap;
  }

  public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterRPMMap() {
    return mShooterRPMMap;
  }

  public synchronized boolean isShooterAtSetpoint(double d, double e) {
    return Util.epsilonEquals(d, e, mShooterAllowableErrorRPM);
  }

  public synchronized boolean isShooterPivotAtSetpoint(
      double current_pivot_angle, double pivot_setpoint) {
    return Util.epsilonEquals(current_pivot_angle, pivot_setpoint, mPivotAllowableErrorDegrees);
  }
}
