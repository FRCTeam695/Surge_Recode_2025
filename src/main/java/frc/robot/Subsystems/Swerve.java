package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.BaseModule;
import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
import frc.robot.RobotContainer;

public class Swerve extends SwerveBase{
    
    public double lastRecordedDistance;
    public Translation2d startTranslation;
    public boolean hasSeenNote;
    public Pose2d notePose;

    public Swerve(AprilTagCamera[] cameras, BaseModule[] modules) {
        super(cameras, modules);

        hasSeenNote = false;
        notePose = new Pose2d();
        startTranslation = new Translation2d();
    }

    public Command snapToPassAngle(Supplier<ChassisSpeeds> speedSupplier){
        return rotateToAngle(()-> isRedAlliance() ? 30 : 150, speedSupplier);
    }


    public Command slowAutonDriveToNote(Supplier<Optional<Double>> yawToNote){
        return
        runOnce(
            ()->{
                startTranslation = getPose().getTranslation();
            }
        )
        .andThen(
            run
            (
                ()-> {
                    if(getPose().getTranslation().getDistance(startTranslation) < 1.5){
                        Optional<Double> yaw = yawToNote.get();
                        if(yaw.isPresent()){
                            double altered_vx, altered_vy;
                            RobotContainer.Driver.rumble(()-> 0.65);
                            altered_vy = 0.04 * yaw.get();
                            double wanted_vel = 0.8;

                            // this means that the wanted velocity is unattainable if the robot strafes towards the note with PID control
                            if(altered_vy > wanted_vel) {
                                altered_vx = 0;
                                altered_vy = Math.copySign(wanted_vel, altered_vy);
                            }
                            else {
                                double ideal_altered_vx_squared = Math.pow(wanted_vel, 2) - Math.pow(altered_vy, 2);
                                altered_vx = Math.sqrt(ideal_altered_vx_squared);
                            }

                            ChassisSpeeds speds = new ChassisSpeeds(-altered_vx, altered_vy, 0);
                            driveFromSpeeds(speds);
                        }
                    }
                    else{
                        driveFromSpeeds(new ChassisSpeeds());
                    }
                }
            )
        );
    }

    public Command teleopDriveToNote(Supplier<Optional<Double>> yawToNote, Supplier<ChassisSpeeds> speedsSupplier){
        return run(
            ()->{
                ChassisSpeeds speeds = speedsSupplier.get();
                Optional<Double> yaw = yawToNote.get();
                if(yaw.isPresent()){
                    double altered_vx, altered_vy;
                    RobotContainer.Driver.rumble(()-> 0.65);
                    altered_vy = 0.04 * yaw.get();
                    double wanted_vel = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

                    // this means that the wanted velocity is unattainable if the robot strafes towards the note with PID control
                    if(altered_vy > wanted_vel) {
                        altered_vx = 0;
                        altered_vy = Math.copySign(wanted_vel, altered_vy);
                    }
                    else {
                        double ideal_altered_vx_squared = Math.pow(wanted_vel, 2) - Math.pow(altered_vy, 2);
                        altered_vx = Math.sqrt(ideal_altered_vx_squared);
                    }

                    ChassisSpeeds speds = new ChassisSpeeds(-altered_vx, altered_vy, 0);
                    driveFromSpeeds(speds);
                }
                else{
                    driveFromSpeeds(speeds);
                }
            }
        );
    }

    

    public Command autonDriveToNote(Supplier<Optional<Double>> yawToNote){
        return
        runOnce(
            ()->{
                startTranslation = getPose().getTranslation();
            }
        )
        .andThen(
            run
            (
                ()-> {
                    if(getPose().getTranslation().getDistance(startTranslation) < 1.5){
                        Optional<Double> yaw = yawToNote.get();
                        if(yaw.isPresent()){
                            double altered_vx, altered_vy;
                            RobotContainer.Driver.rumble(()-> 0.65);
                            altered_vy = 0.04 * yaw.get();
                            double wanted_vel = 1;

                            // this means that the wanted velocity is unattainable if the robot strafes towards the note with PID control
                            if(altered_vy > wanted_vel) {
                                altered_vx = 0;
                                altered_vy = Math.copySign(wanted_vel, altered_vy);
                            }
                            else {
                                double ideal_altered_vx_squared = Math.pow(wanted_vel, 2) - Math.pow(altered_vy, 2);
                                altered_vx = Math.sqrt(ideal_altered_vx_squared);
                            }

                            ChassisSpeeds speds = new ChassisSpeeds(-altered_vx, altered_vy, 0);
                            driveFromSpeeds(speds);
                        }
                    }
                    else{
                        driveFromSpeeds(new ChassisSpeeds());
                    }
                }
            )
        );
    }

    


    public Command rotateToNote(Supplier<ChassisSpeeds> speedSupplier, Supplier<Optional<Double>> yawSupplier){
        return 
        run
        (
            /* EXCECUTE */
            ()-> {
                    Optional<Double> yaw = yawSupplier.get();
                    ChassisSpeeds speeds = speedSupplier.get();
                    if(!yaw.isEmpty()){
                        double gyro_heading = getGyroHeading().getDegrees();
                        double note_yaw = yaw.get();
                        SmartDashboard.putNumber("Z Gyro Heading", gyro_heading);
                        SmartDashboard.putNumber("Z Note Yaw", note_yaw);
                        SmartDashboard.putNumber("Z Setpoint", 180+gyro_heading+note_yaw);
                        speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(gyro_heading-note_yaw);
                    }
                    drive(speeds, true);
                    SmartDashboard.putBoolean("Swerve at Rotation Setpoint", atRotationSetpoint.getAsBoolean());
                 }
        );
    }



    public Command rotateToSpeaker(Supplier<ChassisSpeeds> speedSupplier, Supplier<Optional<Double>> yawSupplier){
        return run
        (
            /* EXCECUTE */
            ()-> {
                    Optional<Double> yaw = yawSupplier.get();
                    ChassisSpeeds speeds = speedSupplier.get();
                    if(!yaw.isEmpty()){
                        speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(getGyroHeading().getDegrees() - yaw.get());
                    }
                    
                    drive(speeds, true);
                 }
        );
    }


    public Command driveToNoteWithPoses(Supplier<Optional<Transform2d>> noteTransform, Supplier<ChassisSpeeds> speedsSupplier){
        return 
            runOnce(
                ()->{
                    hasSeenNote = false;
                }
            )
            .andThen(
                run(
                    ()->{
                        ChassisSpeeds speeds = speedsSupplier.get();
                        Optional<Transform2d> transform = noteTransform.get();
                        if(transform.isPresent()){
                            notePose = currentRobotPose.plus(transform.get());
                            hasSeenNote = true;
                        }
                        if(hasSeenNote){
                            RobotContainer.Driver.rumble(()-> 0.65);
                            double altered_vx, altered_vy;
                            double yaw = notePose.minus(currentRobotPose).getTranslation().getAngle().getDegrees();
                            altered_vy = 0.06 * yaw;
                            double wanted_vel = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    
                            // this means that the wanted velocity is unattainable if the robot strafes towards the note with PID control
                            if(altered_vy > wanted_vel) {
                                altered_vx = 0;
                                altered_vy = Math.copySign(wanted_vel, altered_vy);
                            }
                            else {
                                double ideal_altered_vx_squared = Math.pow(wanted_vel, 2) - Math.pow(altered_vy, 2);
                                altered_vx = Math.sqrt(ideal_altered_vx_squared);
                            }
    
                            speeds.vxMetersPerSecond = -altered_vx;
                            speeds.vyMetersPerSecond = altered_vy;
                        }
                        driveFromSpeeds(speeds);
                    }
                )
        );
    }
}