// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Subsystems.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Map;
import java.util.function.DoubleSupplier;


public class RobotContainer {

  private final VisionManager Vision;
  
  
  private final String[] camNames = {"limelight-shooter"};
    
  // Creates an array of all the swerve modules, pass this into Swerve



  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    Vision = new VisionManager(camNames);
   
  
   

    // configureBindings binds Commands to different button presses (or triggers),
    // Commands are an important part of our programming, here are the docs,
    // https://docs.wpilib.org/en/2021/docs/software/commandbased/index.html
    // I would highly suggest reading all of the links on this page


    // configureDefaultCommands sets default commands to each subsystem, 
    // a default command runs when the subsystem is not otherwise required by a different command, look at the docs for more details,
    // https://docs.wpilib.org/en/2021/docs/software/old-commandbased/commands/default-commands.html

    autoChooser = AutoBuilder.buildAutoChooser();
    sendAutoChooserToDashboard();


  }

  /*
   * I put it in a method so I can't accidentally comment out the auton
   */
  private void sendAutoChooserToDashboard(){
    // This creates our auto chooser and sends it to SmartDashboard, look at pathplanner docs for more details
    // https://pathplanner.dev/home.html
    // autoChooser = AutoBuilder.buildAutoChooser("myauto");
     SmartDashboard.putData("Auto Chooser", autoChooser);
  }

}