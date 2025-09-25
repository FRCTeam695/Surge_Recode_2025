// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //motors
  private SparkFlex shooter1;
  private SparkFlex shooter2;

  //pid controllers
  private SparkClosedLoopController pid1;
  private SparkClosedLoopController pid2;

  //network tables
  NetworkTable table1 = NetworkTableInstance.getDefault().getTable("speedOfMotor1");
  NetworkTable table2 = NetworkTableInstance.getDefault().getTable("speedOfMotor2");

  public ExampleSubsystem() {
    
    //set motors
    shooter1 = new SparkFlex(50, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    shooter2 = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    //set pid controllers
    pid1 = shooter1.getClosedLoopController();
    pid2 = shooter2.getClosedLoopController();

    //set limits!
    SparkMaxConfig config = new SparkMaxConfig();
    
    //will this be successfully configured? is another config needed to be made
    config.closedLoop
    .p(PIDConstants.kP)
    .i(PIDConstants.kI)
    .d(PIDConstants.kD)
    .outputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

    // this line accounts for if the battery voltage is fluctuating
    config.voltageCompensation(11.5);
    //sets break mode
    config.idleMode(IdleMode.kBrake);
    //current limit to 40A
    config.smartCurrentLimit(40);

    //check what safeparams and persistperams means
    shooter1.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    shooter2.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public Command shoot(double speed) {
    return run(
      () -> {
        pid1.setReference(speed, ControlType.kVelocity);
        pid2.setReference(-speed, ControlType.kVelocity);
      }
    );
  }

  public Command stopShoot() {
    return runOnce(
      () -> {
        shooter1.set(0);
        shooter2.set(0);
      }
    );
  }


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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
