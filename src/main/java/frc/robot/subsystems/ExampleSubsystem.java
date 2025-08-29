// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SparkFlex shooter1;
  private SparkFlex shooter2;

  public ExampleSubsystem() {
    shooter1 = new SparkFlex(50, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    shooter2 = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  }

  public Command shoot() {
    return runOnce(
      () -> {
        shooter1.set(1);
        shooter2.set(1);
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
