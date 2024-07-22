// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armAngle = 0;
    public double armVolts = 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setArmAngle(double targetAngle) {}

  public default void stopArm() {}

  public default void updateInputs(ArmIOInputs inputs) {}
}
