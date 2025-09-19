// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SparkFlex shooter1;
  private SparkFlex shooter2;
  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
  NetworkTable table1 = NetworkTableInstance.getDefault().getTable("speedOfMotor1");
  NetworkTable table2 = NetworkTableInstance.getDefault().getTable("speedOfMotor2");

  public ExampleSubsystem() {
    shooter1 = new SparkFlex(50, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    shooter2 = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  }

  public Command shoot(double speed) {
    return run(
      () -> {
        double measurement = shooter1.getEncoder().getVelocity();
        double output = pid.calculate(measurement, speed);
        shooter1.set(-1 * output);
        shooter2.set(output);

        table1.getEntry("speedOfMotor1").setDouble(measurement);
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
