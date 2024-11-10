package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.BaseModule;
import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
import frc.robot.RobotContainer;

public class Swerve extends SwerveBase{

    private Pose2d notePose;
    private boolean hasSeenNote;

    public Swerve(AprilTagCamera[] cameras, BaseModule[] modules) {
        super(cameras, modules);

        notePose = new Pose2d();
        hasSeenNote = false;
    }

    public Command driveToNote(Supplier<Optional<Transform2d>> noteTransform, Supplier<ChassisSpeeds> speedsSupplier){
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