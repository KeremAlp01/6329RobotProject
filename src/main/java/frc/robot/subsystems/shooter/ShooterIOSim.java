// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getKrakenX60(2), 1.0 / 1.25, 0.003);
  private PIDController pid = new PIDController(0.001, 0, 0);

  private double leftTargetRPM = 0;
  private double rightTargetRPM = 0;
  private double volts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    sim.update(0.02);

    inputs.leftRPM = sim.getAngularVelocityRPM();
    inputs.rightRPM = sim.getAngularVelocityRPM();
    SmartDashboard.putNumber("LeftRPM", inputs.leftRPM);
    SmartDashboard.putNumber("rightRPM", inputs.rightRPM);
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setTargetRPM(double leftRPM, double rightRPM, double ffVolts) {
    leftTargetRPM = leftRPM;
    rightTargetRPM = rightRPM;
    volts =
        MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM(), leftRPM) + ffVolts, -12.0, 12.0);
    sim.setInputVoltage(volts);
  }

  @Override
  public double getLeftTargetRPM() {
    return leftTargetRPM;
  }

  @Override
  public double getRightTargetRPM() {
    return rightTargetRPM;
  }
}
