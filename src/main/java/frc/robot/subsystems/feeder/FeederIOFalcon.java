// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class FeederIOFalcon implements FeederIO{
    private final TalonFX feederMotor = new TalonFX(31, "Canbus");

    private final StatusSignal<Double> feederVolts = feederMotor.getMotorVoltage();

    public FeederIOFalcon(){
        configFeederMotor(feederMotor);
         BaseStatusSignal.setUpdateFrequencyForAll(50.0,feederVolts);

    }

    public void configFeederMotor(TalonFX talon){

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
    public void updateInputs(FeederIOInputs inputs){
        BaseStatusSignal.refreshAll(feederVolts);
        inputs.feederVolts = feederMotor.getMotorVoltage().getValueAsDouble();
        
    }

    @Override
    public void setVoltage(double votlage){
        feederMotor.setVoltage(votlage);
    }

}
