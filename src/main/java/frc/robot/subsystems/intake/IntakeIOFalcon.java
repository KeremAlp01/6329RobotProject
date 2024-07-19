// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class IntakeIOFalcon implements IntakeIO{
    //motor
    private final TalonFX intakeMotor = new TalonFX(11);
    //StatusSignals
    private StatusSignal<Double> intakeMotorVolts = intakeMotor.getMotorVoltage();

    public IntakeIOFalcon(){
    configIntakeTalonFX(intakeMotor);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeMotorVolts);
    }

    public void configIntakeTalonFX(TalonFX talon){
        talon.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration(); 

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        BaseStatusSignal.refreshAll(intakeMotorVolts);

        intakeMotorVolts = intakeMotor.getMotorVoltage();
    }
    @Override
    public void setIntakeVolts(double voltage){
        intakeMotor.setVoltage(voltage);
    }

}
