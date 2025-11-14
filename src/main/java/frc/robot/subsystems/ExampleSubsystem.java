
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.NetworkTableEntry;

import com.revrobotics.spark.SparkClosedLoopController;

public class ExampleSubsystem extends SubsystemBase {

  //motors
  private SparkFlex shooter1;
  private SparkFlex shooter2;

  //pid controllers
  private SparkClosedLoopController pid1;
  private SparkClosedLoopController pid2;

  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;

  private NetworkTableInstance inst;
  private NetworkTable table;
  private NetworkTableEntry velocityEntry;

  private double currentRPM;


  public ExampleSubsystem() {
    
  shooter1 = new SparkFlex(50, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  shooter2 = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //set motors
     encoder1 = shooter1.getEncoder();
     encoder2 = shooter2.getEncoder();

    //set pid controllers
    pid1 = shooter1.getClosedLoopController();
    pid2 = shooter2.getClosedLoopController();

    //configuring pid controller for shooter1
    SparkFlexConfig config1 = new SparkFlexConfig();
    config1.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(PIDConstants.kP)
      .i(PIDConstants.kI)
      .d(PIDConstants.kD)
      .velocityFF(PIDConstants.ff)
      .outputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    config1.voltageCompensation(11.5);
    config1.idleMode(IdleMode.kBrake);
    config1.smartCurrentLimit(40);

    config1.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    //configuring pid controller for shooter2
    SparkFlexConfig config2 = new SparkFlexConfig();
    config2.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(PIDConstants.kP)
      .i(PIDConstants.kI)
      .d(PIDConstants.kD)
      .velocityFF(PIDConstants.ff)
      .outputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

    config2.voltageCompensation(11.5);
    config2.idleMode(IdleMode.kBrake);
    config2.smartCurrentLimit(40);

    config2.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    //applies configurements onto motors
    shooter1.configure(config1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    shooter2.configure(config2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("velocity");
    velocityEntry = table.getEntry("currentRPM");
  }

  //shoot to desired speed
  public Command shoot(double speedRPM) {
    return run(
      () -> {
        pid1.setReference(speedRPM, ControlType.kVelocity);
        pid2.setReference(-speedRPM, ControlType.kVelocity);
        //absolute encoder?
        currentRPM = shooter1.getAbsoluteEncoder().getVelocity();
        velocityEntry.setDouble(currentRPM);
      }
    );  
  }

  //sets speed to zero, stops shooting
  public Command stopShoot() {
    return runOnce(
      () -> {
        shooter1.set(0);
        shooter2.set(0);
      }
    );
  }

  /*public double getVelocity() {
    return shooter1.getEncoder().getVelocity();
  }*/

   /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("velocity", encoder1.getVelocity());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
