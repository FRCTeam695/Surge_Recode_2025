package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpBar extends SubsystemBase {

  private SparkMax ampBarMotor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController controller;

  public AmpBar() {
        ampBarMotor = new SparkMax(57, MotorType.kBrushless); 
        
        ClosedLoopConfig controllerConfig = new ClosedLoopConfig();
        controllerConfig.p(0.21);
        
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.smartCurrentLimit(0, 15, 0);
        sparkConfig.idleMode(IdleMode.kBrake);
        sparkConfig.apply(controllerConfig);
        ampBarMotor.configure(sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        encoder = ampBarMotor.getEncoder();
        encoder.setPosition(0);
        controller = ampBarMotor.getClosedLoopController();
  }

  public Command closedLoopControl(double setpoint){
    return new FunctionalCommand(
    ()-> {
        },
    ()-> controller.setReference(setpoint, SparkMax.ControlType.kPosition),
    interrupted-> {},
    ()-> Math.abs(encoder.getPosition() - setpoint) < 0.1,
    this
);
  }

  public Command deploy(){
    return closedLoopControl(12);
  }

  public Command retract(){
    return closedLoopControl(0);
  }

  public boolean isDeployed(){
    return Math.abs(encoder.getPosition()) >= 0.1;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("amp bar encoder", encoder.getPosition());
  }
}

