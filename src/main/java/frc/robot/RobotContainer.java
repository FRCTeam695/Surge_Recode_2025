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
import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Subsystems.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Map;
import java.util.function.DoubleSupplier;


public class RobotContainer {

  public final Swerve Swerve;
  private final VisionManager Vision;
  final static Arm Arm = new Arm();
  private final AmpBar AmpBar;
  private final Shooter Shooter;
  private final Intake Intake;
  private final LEDs LEDs;
  
  private final String[] camNames = {"limelight-speaker"};
    
  // Creates an array of all the swerve modules, pass this into Swerve
  private final TalonFXModule[] modules = new TalonFXModule[] 
          {
            new TalonFXModule(Constants.Swerve.FRONT_RIGHT_DRIVE_ID, Constants.Swerve.FRONT_RIGHT_TURN_ID, Constants.Swerve.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_RIGHT_CANCODER_ID, 0),
            new TalonFXModule(Constants.Swerve.FRONT_LEFT_DRIVE_ID, Constants.Swerve.FRONT_LEFT_TURN_ID, Constants.Swerve.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.FRONT_LEFT_CANCODER_ID, 1),
            new TalonFXModule(Constants.Swerve.BACK_LEFT_DRIVE_ID, Constants.Swerve.BACK_LEFT_TURN_ID, Constants.Swerve.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_LEFT_CANCODER_ID, 2),
            new TalonFXModule(Constants.Swerve.BACK_RIGHT_DRIVE_ID, Constants.Swerve.BACK_RIGHT_TURN_ID, Constants.Swerve.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS, Constants.Swerve.BACK_RIGHT_CANCODER_ID, 3)
          };

  // Creates an Xbox Controller Object, the first parameter relates to the port on the laptop
  public static final EnhancedCommandController Driver = new EnhancedCommandController(0);
  private final EnhancedCommandController Operator = new EnhancedCommandController(1);

  private final Trigger isReadyToShoot;
  private final Trigger ampBarDeployed;
  private final Trigger isMovingTooFast;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    Vision = new VisionManager(camNames);
    Swerve = new Swerve(camNames, modules);
    AmpBar = new AmpBar();
    Shooter = new Shooter();
    Intake = new Intake();
    LEDs = new LEDs();

    isMovingTooFast = new Trigger(()-> Swerve.getLatestSpeed() > 1);

    isReadyToShoot = Swerve.atRotationSetpoint.and(Shooter.atVelocitySetpoint).and(Arm.atPositionSetpoint).and(isMovingTooFast.negate()).and(Vision.tagInSight);
    ampBarDeployed = new Trigger(AmpBar::isDeployed);


    new EventTrigger("Intake").onTrue(
      deadline(
      intake()
      //, drive to note with poses
      )
      
    );
    NamedCommands.registerCommand("Mid First Shot",
      deadline(
      Arm.goToPosition(()-> Constants.Arm.SHOOT_POSITION_RADIANS),
      Shooter.runVelocity(()-> 2222.0/5700)
      ).andThen(shoot(()-> 2222).withTimeout(2).andThen(LEDs.turnColorOff()))
    );



    NamedCommands.registerCommand("Intake Note", intake());
    NamedCommands.registerCommand("Shoot Note", shoot(()->2222).withTimeout(2).andThen(LEDs.turnColorOff()));
    NamedCommands.registerCommand("Shoot Hard", shoot(()-> 4000).withTimeout(2).andThen(LEDs.turnColorOff()));
    NamedCommands.registerCommand("Podium Shot", shoot(()-> 3000).withTimeout(2).andThen(LEDs.turnColorOff()));
    NamedCommands.registerCommand("Podium Shot Position", Arm.goToPosition(()-> Math.toRadians(39)));
    NamedCommands.registerCommand("Side Shot Position", Arm.goToPosition(()-> Math.toRadians(0.88)));
    NamedCommands.registerCommand("Source Shot Position", Arm.goToPosition(()-> 1));
    NamedCommands.registerCommand("Shoot Position", Arm.goToPosition(()-> Constants.Arm.SHOOT_POSITION_RADIANS));
    NamedCommands.registerCommand("Shoot Hard Position", Arm.goToPosition(()-> Units.degreesToRadians(41)));
    NamedCommands.registerCommand("Auto Intake", deadline(intake(), Swerve.autonDriveToNote(()-> Vision.getYawToNote())));
    NamedCommands.registerCommand("Slow Auto Intake", deadline(intake(), Swerve.slowAutonDriveToNote(()-> Vision.getYawToNote())));
    NamedCommands.registerCommand("Auto Shoot", autoShoot());
    NamedCommands.registerCommand("Prep Shot", prepAutonShot());
    

    // configureBindings binds Commands to different button presses (or triggers),
    // Commands are an important part of our programming, here are the docs,
    // https://docs.wpilib.org/en/2021/docs/software/commandbased/index.html
    // I would highly suggest reading all of the links on this page
    configureBindings();

    // configureDefaultCommands sets default commands to each subsystem, 
    // a default command runs when the subsystem is not otherwise required by a different command, look at the docs for more details,
    // https://docs.wpilib.org/en/2021/docs/software/old-commandbased/commands/default-commands.html
    configureDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    sendAutoChooserToDashboard();

    // SmartDashboarding subsystems allow you to see what commands they are running
    SmartDashboard.putData("Swerve Subsystem", Swerve);
    SmartDashboard.putData("Arm Subsystem", Arm);
    SmartDashboard.putData("Shooter Subsystem", Shooter);
    SmartDashboard.putData("Intake Subsystem", Intake);
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

  private void configureBindings() {
    Operator.a().onTrue(Arm.goToPosition(()-> Constants.Arm.INTAKE_POSITION_RADIANS));
    Operator.b().whileTrue(Intake.runIndexerToSpeed(-0.2));
    Operator.x().onTrue(Arm.goToPosition(()-> 0.44).andThen(Shooter.setScoringStatus("climb")));
    Operator.y().whileTrue(Arm.goToPosition(()-> Constants.Arm.INTAKE_POSITION_RADIANS).andThen(Intake.runIntakeAndIndexerPercent(-1)));


    
    Driver.b().onTrue(Swerve.runWheelCharacterization());
    Driver.y().toggleOnTrue(visionIntake().andThen(autoShoot()).andThen(visionIntake()).andThen(autoShoot()).andThen(visionIntake()).andThen(autoShoot()).andThen(visionIntake()).andThen(autoShoot()));
    Driver.back().onTrue(Swerve.resetGyro());
    Driver.a().onTrue(Shooter.setScoringStatus("amp")
      .andThen(
        parallel(
          Swerve.rotateToAngle(()-> -90., Driver::getRequestedChassisSpeeds).until(()-> !Shooter.getScoringStatus().equals("amp")), 
          Arm.goToPosition(()-> Constants.Arm.AMP_POSITION_RADIANS))));
    Driver.x().onTrue(Shooter.setScoringStatus("stockpile")
                    .andThen(
                      parallel(
                      Swerve.snapToPassAngle(Driver::getRequestedChassisSpeeds),
                      Arm.goToPosition(()-> Constants.Arm.STOCKPILE_POSITION_RADIANS),
                      Shooter.runVelocity(()-> Constants.Shooter.STOCKPILE_RPM))));
    Driver.leftTrigger(0.2).onTrue(
          Arm.goToPosition(()-> Constants.Arm.SHOOT_POSITION_RADIANS)
          .andThen(shoot(()->2222))
          .andThen(Shooter.setScoringStatus("intake"))
    );
    Driver.rightBumper().onTrue(shoot(()->1000));
    Driver.leftBumper().onTrue(
      deadline(
        intake(), 
        Swerve.driveToNoteWithPoses(Vision::getRobotToNote, Driver::getRequestedChassisSpeeds))
    );
    Driver.rightTrigger(0.2).onTrue(shootNoteCommand().andThen(Arm.goToPosition(()-> Constants.Arm.INTAKE_POSITION_RADIANS)));

    // Driver.leftBumper().onTrue
    // (
    //     deadline
    //     (
    //     intake(),
    //     Swerve.rotateToNote(()-> Driver.getRequestedChassisSpeeds(), ()-> Vision.getYawToNote())
    //     )
    //     .until(()-> Operator.getHID().getRightBumperButtonPressed()).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    // );




    Intake.beamIsBroken.onTrue(Driver.rumble(()-> 0.4).withTimeout(0.25));
    
    //disable movement that could break the amp bar if amp bar is deployed
    ampBarDeployed.whileTrue(
            run(()-> {
                    Swerve.drive(new ChassisSpeeds(), true);
                 }));


    //Driver.b().onTrue(Intake.runIndexerToSpeed(-0.1).withTimeout(0.5).andThen(Intake.runIndexerToSpeed(0)));
    // Shooter.readyToIntake.and(Vision.canIntakeNote).and(()-> !Driver.getHID().getLeftBumper())
    // .onTrue(
    //   deadline
    //     (
    //       intake(),
    //       Swerve.teleopDriveToNote(()-> Vision.getYawToNote(), ()-> Driver.getRequestedChassisSpeeds())
    //     ).until(()-> Operator.getHID().getRightBumper())
    // );
    //Driver.a().onTrue(Swerve.runWheelCharacterization());
  }

  public void configureDefaultCommands(){

    Shooter.setDefaultCommand(Shooter.runVelocity(()-> 0).withName("Shooter Default Command"));
    Intake.setDefaultCommand(Intake.runIntakeAndIndexerPercent(0.0));
    AmpBar.setDefaultCommand(AmpBar.retract());

    
    //This is the Swerve subsystem default command, this allows the driver to drive the robot
    Swerve.setDefaultCommand
      (
        run
          (
            ()->{
              //double ti = System.nanoTime();
              Swerve.teleopDefaultCommand(
                Driver::getRequestedChassisSpeeds,
                true
              );
              //double tf = System.nanoTime();
              //System.out.println((tf-ti)/1000000);
              }
              ,
              Swerve
          ).withName("Swerve Default Command")
      );
      
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    //return new WaitCommand(5);
    return new PathPlannerAuto("TestingAuto").andThen(()-> Swerve.stopModules());
    //return autoChooser.getSelected().andThen(()-> Swerve.stopModules());
  }

  private Command visionIntake(){
    return deadline
        (
        intake(),


        
        Swerve.rotateToNote(()-> Driver.getRequestedChassisSpeeds(), ()-> Vision.getYawToNote())
        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }


  private Command intake() {
    return 

    // makes sure arm is in the correct position
    Arm.goToPosition(()-> Constants.Arm.INTAKE_POSITION_RADIANS)
    .andThen
    (
    
    // (runs the shooter to 0 velocity and run intake and indexer) until we get a beambreak
    race(

        // LED is for driver preference, so he knows when the robot is still intaking
        LEDs.setColorToGreen()
        .andThen
        (
          Intake.runIntakeAndIndexerPercent(0.5)
          .until
          (
            Intake::getBeamBreak
          )
        ),
        

        Shooter.runVelocity(()-> 0)
      )
    )

    // LED to orange means driver can drive away, won't effect the intake, we have the note
    .andThen
    (
      LEDs.setColorToOrange()
    )
    
    .andThen
    (
      Shooter.setScoringStatus("speaker")
    )


    // runs the indexer backwards using closed loop control,
    // we do this because the shooter wheels need room to spin up to speed
    .andThen
    (
      Intake.indexerClosedLoopControl(0.2)
      .until
      (
        Intake::getBeamMade
      )
    )

    .andThen
    (
      Intake.setNoteStatus(true)
    )

    // moves the arm to shoot position, 
    // we default to shoot position until told otherwise by driver
    .andThen
    (
      Arm.goToPosition(()-> Constants.Arm.SHOOT_POSITION_RADIANS)
    )

    // the withName is for named commands
    .withName("Intake Note")
    ;
  }




  private Command shoot(DoubleSupplier RPM) {
    return race(
      
      // run the shooter wheels to speed
      Shooter.runVelocity(()-> (RPM.getAsDouble()/5700)),

      // wait for shooters to get up to speed
      waitUntil(Shooter::isRunning)
        .andThen(waitUntil(Shooter::shooterIsUpToSpeed))
        .andThen(

            race(

              // the shooters are now up to speed, so we run the indexers forward full speed
              Intake.runIndexerToSpeed(1),

              // the following logic watches for a velocity dip in the shooter wheels
              waitUntil(Shooter::shooterIsNotUpToSpeed)
                .andThen(new WaitCommand(0.1))
                .andThen(
                  waitUntil(Shooter::shooterIsUpToSpeed)
                      )
                )
        )
            
    )
    .andThen(Shooter.setScoringStatus("intake"));
  }



  private Command shootNoteCommand(){
    return new SelectCommand<>(

      Map.ofEntries(
        Map.entry("amp", 

          Arm.goToPosition(()-> Constants.Arm.AMP_POSITION_RADIANS)
          .andThen(
            parallel
            (
              AmpBar.deploy(),
              shoot(()-> 1000)
            ).withTimeout(1)
          )
          .andThen(new WaitCommand(0.4))
          .andThen(AmpBar.retract())
          .andThen(LEDs.turnColorOff()).andThen(Shooter.setScoringStatus("intake"))
          ),
          
        Map.entry("speaker", 
          // use getHID in order to not run into loop overruns, below is relevent github issue
          // https://github.com/wpilibsuite/allwpilib/issues/5903
          autoShoot().andThen(Shooter.setScoringStatus("intake"))
          ),


        Map.entry("stockpile", 
          Arm.goToPosition(()-> Constants.Arm.STOCKPILE_POSITION_RADIANS)
          .andThen(shoot(()-> Constants.Shooter.STOCKPILE_RPM)).withTimeout(2).andThen(Shooter.setScoringStatus("intake")))
          
      ),

    ()-> Shooter.getScoringStatus());
  }

  public Command prepAutonShot(){
    return parallel
          (
            Shooter.runVelocity(()-> (0.8 *Vision.getShooterRPMToSpeaker()/5700)), 
            Arm.goToPositionNoEnd(Vision::getArmPitchToSpeaker)
          );
  }


  public Command autoShoot(){
    //return Shooter.runVelocity(()-> (3300/5700));
    //return Shooter.runVelocity(()-> (0.57894736));
    return 
        deadline(
          parallel
            (
              Swerve.rotateToSpeaker(Driver::getRequestedChassisSpeeds, Vision::getYawToSpeaker),
              Shooter.runVelocity(()-> (Vision.getShooterRPMToSpeaker()/5700)) 
              //Arm.goToPositionNoEnd(Vision::getArmPitchToSpeaker)//,
            ).until(isReadyToShoot)
          .andThen
              (
              deadline(
              shoot(()->Vision.getShooterRPMToSpeaker()),
              Swerve.rotateToSpeaker(Driver::getRequestedChassisSpeeds, Vision::getYawToSpeaker)
              )
            ),

          Arm.goToPositionNoEnd(Vision::getArmPitchToSpeaker)
        )
          // .andThen(new PrintCommand("just finished shooting"))x
          .andThen(Shooter.setScoringStatus("intake"));
  }

    public Command autonomousShoot(){
    return 
          deadline(
            parallel
              (
                Swerve.rotateToSpeaker(()-> new ChassisSpeeds(), Vision::getYawToSpeaker),
                Shooter.runVelocity(()-> (Vision.getShooterRPMToSpeaker()/5700))
                //,
              ).until(isReadyToShoot)
            .andThen
              (
                deadline(
                shoot(()->Vision.getShooterRPMToSpeaker()),
                Swerve.rotateToSpeaker(Driver::getRequestedChassisSpeeds, Vision::getYawToSpeaker)
                )
              ),

            Arm.goToPositionNoEnd(Vision::getArmPitchToSpeaker)
          )
          // .andThen(new PrintCommand("just finished shooting"))x
          .andThen(Shooter.setScoringStatus("intake"));
  }

}