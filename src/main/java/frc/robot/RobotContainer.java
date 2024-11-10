// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.BaseModule;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
import frc.BisonLib.BaseProject.Vision.Limelight.LLAprilTagCamera;

import static edu.wpi.first.wpilibj2.command.Commands.*;


public class RobotContainer {

  public final SwerveBase Swerve;

  // Creates an array that holds all our pose estimation cameras, pass this into Swerve
  private final AprilTagCamera[] cameras = new AprilTagCamera[] 
          {
            // new PVAprilTagCamera("speakerCamera", 0., 
            //   new Transform3d
            //   (
            //     Units.inchesToMeters(13.5),
            //     0., 
            //     Units.inchesToMeters(8), 
            //     new Rotation3d(0, -65, 0)
            //   )
            // )
            new LLAprilTagCamera("limelight-speaker", 0.)
          };
    
  // Creates an array of all the swerve modules, pass this into Swerve
  private final BaseModule[] modules = new BaseModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  // Creates an Xbox Controller Object, the first parameter relates to the port on the laptop
  public static final EnhancedCommandController Driver = new EnhancedCommandController(0);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    Swerve = new SwerveBase(cameras, modules);

    // configureBindings binds Commands to different button presses (or triggers),
    // Commands are an important part of our programming, here are the docs,
    // https://docs.wpilib.org/en/2021/docs/software/commandbased/index.html
    // I would highly suggest reading all of the links on this page
    configureBindings();

    // configureDefaultCommands sets default commands to each subsystem, 
    // a default command runs when the subsystem is not otherwise required by a different command, look at the docs for more details,
    // https://docs.wpilib.org/en/2021/docs/software/old-commandbased/commands/default-commands.html
    configureDefaultCommands();
    sendAutoChooserToDashboard();

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);
  }

  /*
   * I put it in a method so I can't accidentally comment out the auton
   */
  private void sendAutoChooserToDashboard(){
    // This creates our auto chooser and sends it to SmartDashboard, look at pathplanner docs for more details
    // https://pathplanner.dev/home.html
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Driver.back().onTrue(Swerve.resetGyro());
  }

  public void configureDefaultCommands(){
    // This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        run
          (
            ()-> 
              Swerve.teleopDefaultCommand(
                Driver::getRequestedChassisSpeeds,
                true
              )
              ,
              Swerve
          ).withName("Swerve Drive Command")
      );
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    //return new WaitCommand(5);
    return autoChooser.getSelected().andThen(()-> Swerve.stopModules());
  }
}
