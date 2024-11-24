package frc.robot.Subsystems;

import java.util.Optional;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
import frc.BisonLib.BaseProject.Vision.VisionManagerBase;
import frc.BisonLib.BaseProject.Vision.Limelight.LLIntakeCamera;
import frc.BisonLib.BaseProject.Vision.Limelight.LimelightHelpers;
import frc.robot.Constants;

public class VisionManager extends VisionManagerBase{
    public Optional<Double> distanceToSpeaker;
    public Optional<Double> yawToSpeaker;
    public Optional<Double> pitchToSpeaker;
    public double latency;

    public static Optional<Double> yawToNote;//CCW negative, CW positive, given by Limelight
    public static Optional<Double> pitchToNote;
    public static Optional<Translation2d> robotToNote;
    
    public double armPitch;
    public double shooterRPM;
    public boolean canSeeNote;
    public boolean canSeeTag;

    private InterpolatingDoubleTreeMap armTable;
    private InterpolatingDoubleTreeMap shooterTable;


    public Trigger tagInSight;
    public Trigger canIntakeNote;
    public LLIntakeCamera intakeCamera;

    public VisionManager(AprilTagCamera[] cameras, LLIntakeCamera intakeCamera){
        super(cameras);

        this.intakeCamera = intakeCamera;

        /*
         * MAKE SURE TO FIND THE MAX WORKABLE RANGE TO SHOOT FROM
         */
        //armTable.put(distance to april tag, arm pitch);
        armTable = new InterpolatingDoubleTreeMap();
        armTable.put(25.76, Constants.Arm.SHOOT_POSITION_RADIANS);
        armTable.put(22.12, Math.toRadians(52));
        armTable.put(14.57, Math.toRadians(48));
        armTable.put(11.11, Math.toRadians(45));
        //armTable.put(6.935, Math.toRadians(44.1));
        armTable.put(3.05, Math.toRadians(41));
        armTable.put(2.72, Math.toRadians(37));
        armTable.put(1.34, Math.toRadians(37));
        armTable.put(-1.92, Math.toRadians(35));
        armTable.put(-3.1, Math.toRadians(34));
        armTable.put(-3.97, Math.toRadians(32.4));





        // armTable.put(0.32, Math.toRadians(32));
        // armTable.put(-0.76, Math.toRadians(36.2));
        // armTable.put(-1.25, Math.toRadians(36));
        // armTable.put(-2.66, Math.toRadians(34.8));



        //armTable.put(-1.92, Math.toRadians(36));
        // armTable.put(21.105, Math.toRadians(52));
        // armTable.put(15.48, Math.toRadians(49));
        // armTable.put(11.32, Math.toRadians(47));
        // armTable.put(7.05, Math.toRadians(44.5));
        // armTable.put(4.62, Math.toRadians(42.5));
        // armTable.put(1.77, Math.toRadians(40.1));
        // armTable.put(-0.98, Math.toRadians(37.7));
        // armTable.put(-2.62, Math.toRadians(35));
        // armTable.put(-4.33, Math.toRadians(33));
 
 
        //shooterTable.put(distance to april tag,  RPM);
        shooterTable = new InterpolatingDoubleTreeMap();
        shooterTable.put(22.12, 4050.);
        shooterTable.put(14.57, 4100.);
        shooterTable.put(11.11, 4150.);
        shooterTable.put(6.935, 4200.);
        shooterTable.put(3.05, 5000.);
        shooterTable.put(0.32, 5000.);
        shooterTable.put(-3.97, 5000.);     
        

        // shooterTable.put(-0.76, 4350.);
        // shooterTable.put(-1.25, 4350.);


        // shooterTable.put(21.105, 4000.);
        // shooterTable.put(15.48, 4050.);
        // shooterTable.put(11.32, 4150.);
        // shooterTable.put(7.05, 4200.);
        // shooterTable.put(4.62, 4250.);
        // shooterTable.put(1.77, 4300.);
        // shooterTable.put(3.17, 4350.);

        distanceToSpeaker = Optional.empty();

        //isWithinShootingRange = new Trigger(()-> withinShootingRange);
        canIntakeNote = new Trigger(()-> canSeeNote);
        tagInSight = new Trigger(()-> canSeeTag);
    }

    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
        return false;
    }

    public int getSpeakerID(){
        if(isRedAlliance()) return 4;
        else return 7;
    }

    private Optional<Double> calculateYawToSpeaker(){
        Double yaw = cameras[0].getYawToTag(getSpeakerID());
        if(yaw == null) return Optional.empty();
        return Optional.of(yaw);
    }

    public Optional<Double> getPitchToSpeakerTag(){
        Double pitch = cameras[0].getPitchToTag(getSpeakerID());
        if(pitch == null){
            return Optional.empty();
        }
        else{
            return Optional.of(pitch);
        }
    }


    // doesnt actually turn them on, i just dont want them to turn on
    public Command turnOnLimelightLEDs(){
        return runOnce(()-> LimelightHelpers.setPipelineIndex("limelight-intake", 0));
    }

    public Command turnOffLimelightLEDs(){
        return runOnce(()-> LimelightHelpers.setPipelineIndex("limelight-intake", 0));
    }


    public boolean canSeeNote(){
        return canSeeNote;
    }


    public double getDistanceToSpeakerTag(){
        return distanceToSpeaker.get();
    }

    public Optional<Double> getYawToNote(){
        return yawToNote;
    }

    public Optional<Double> getPitchToNote(){
        return pitchToNote;
    }

    private Translation2d getRobotTranslationToNote(){//robot to note (positive is forward, right)
        Translation2d noteTranslation;

        //distance from target
        //https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/5 
        
        double distance = (Constants.Vision.NOTE_HEIGHT - Constants.Vision.CAMERA_HEIGHT) / 
                    Math.tan(Math.toRadians(Constants.Vision.MOUNT_PITCH + pitchToNote.get())) / 
                    Math.cos(Math.toRadians(Constants.Vision.MOUNT_YAW + yawToNote.get()));
        
        //angle  
        Rotation2d angle = new Rotation2d(Math.toRadians(90 +  (-yawToNote.get())) );//flip the yaw in order to have ccw as positive
        // SmartDashboard.putNumber("angle", angle.getDegrees());
        // SmartDashboard.putNumber("yaw", yawToNote.get());
        
        noteTranslation = new Translation2d(distance, angle);
        return noteTranslation;
    }

    public boolean isNoteCutOffPitch(){
        if(pitchToNote.get() < -11.83){
            return true;
        }
        return false;
    }

    public boolean isNoteCutOffYaw(){
        if(!canSeeNote){//if cannot see the note, return true that note is cut off
            return true;
        }
        double a = 2.13542;
        double b = -38.89291;
        if(pitchToNote.get() > a*Math.abs(yawToNote.get()) + b){ //if note is within the regression
            return false;
        } else {
            return true; 
        }
    }
    //tranlsation holds the values for new yaw and new pitch
    //x is yaw, y is pitch

    public Translation2d getCorrectCutoffNote(){
        double newPitch = pitchToNote.get();
        double newYaw = yawToNote.get();
        double a = 2.13542;
        double b = -38.89291;
        if (isNoteCutOffPitch()) {
            double diff = Math.abs(pitchToNote.get() + 11.83);
            newPitch = pitchToNote.get() - diff;
        }


        if (isNoteCutOffYaw()) {
            double cutoffYaw = (newPitch - b) / a;
            double diff = Math.abs( cutoffYaw - Math.abs(newYaw) );
            if(newYaw > 0){
                newYaw = newYaw + diff;
            }else{
                newYaw = newYaw - diff;
            }
        }
        SmartDashboard.putNumber("corrected Yaw", newYaw);
        SmartDashboard.putNumber("corrected Pitch", newPitch);
        return new Translation2d(newYaw, newPitch);
    }
    public Translation2d getRobotToCutoffNoteCorrection(){
        Translation2d note = getCorrectCutoffNote();
        double m_yaw = note.getX();
        double m_pitch = note.getY();

        Translation2d noteTranslation;

        //distance from target
        //https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/5 
        
        double distance = (Constants.Vision.NOTE_HEIGHT - Constants.Vision.CAMERA_HEIGHT) / 
                    Math.tan(Math.toRadians(Constants.Vision.MOUNT_PITCH + m_pitch)) / 
                    Math.cos(Math.toRadians(Constants.Vision.MOUNT_YAW + m_yaw));
        
        //angle  
        Rotation2d angle = new Rotation2d(Math.toRadians(90 +  (-m_yaw)) );//flip the yaw in order to have ccw as positive
         SmartDashboard.putNumber("angle", angle.getDegrees());
        // SmartDashboard.putNumber("yaw", yawToNote.get());
        
        noteTranslation = new Translation2d(distance, angle);
        SmartDashboard.putNumber("fwd correct", noteTranslation.getY());
        SmartDashboard.putNumber("dist", distance);
        
        return noteTranslation;

    }

    // public Translation2d robotToCutoffNoteregression(){//OUTDATED
    //     double C = -4.438623;
    //     double yawC =0.734597;
    //     double pitchC = 0.0581734;
    //     double STR =  C + yawC * Math.abs(yawToNote.get()) + pitchC * Math.abs(pitchToNote.get());
    //     //flip if negative
    //     if(yawToNote.get() < 0){
    //         STR *= -1;
    //     }
        
    //     //flat approximazation
    //     double FWD = STR / Math.tan( Math.toRadians(yawToNote.get()));
    //     Translation2d note = new Translation2d(STR,FWD);
    //     SmartDashboard.putNumber("cutoff calculated str", note.getX());
    //     SmartDashboard.putNumber("cutoff calculated fwd", note.getY());
    //     return note;
    // }

    
    public double getArmPitchToSpeaker(){
        return armPitch;
    }


    public double getShooterRPMToSpeaker(){
        return shooterRPM;
    }

    public Optional<Double> getYawToSpeaker(){
        return yawToSpeaker;
    }

    public Optional<Translation2d> getRobotToNote(){
        return robotToNote;
    }


    @Override
    public void periodic(){ 
        cameras[0].loadLatestResult();
        //distanceToSpeaker = calculateDistanceToSpeaker();
        pitchToSpeaker = getPitchToSpeakerTag();
        yawToSpeaker = calculateYawToSpeaker();
        canSeeNote = intakeCamera.canSeeTarget();
        latency = intakeCamera.getLatency();

        if(pitchToSpeaker.isPresent() && yawToSpeaker.isPresent()){
            //shooterRPM = 2222;
            //armPitch = Constants.Arm.SHOOT_POSITION_RADIANS;
            canSeeTag = true;
            shooterRPM = shooterTable.get(pitchToSpeaker.get());
            armPitch = armTable.get(pitchToSpeaker.get());

            SmartDashboard.putNumber("pitch to speaker", pitchToSpeaker.get());
            SmartDashboard.putNumber("yaw to speaker", yawToSpeaker.get());
        }
        else{
            canSeeTag = false;
        }


        if(canSeeNote){
            yawToNote = Optional.of(intakeCamera.getYawToTarget());
            SmartDashboard.putNumber("yaw to note", yawToNote.get());
            pitchToNote = Optional.of(intakeCamera.getPitchToTarget());
            SmartDashboard.putNumber("Pitch to note", pitchToNote.get());
            if(!isNoteCutOffYaw()){
                robotToNote = Optional.of(getRobotTranslationToNote());
            } else{
                robotToNote = Optional.of(getRobotToCutoffNoteCorrection());
            }
            SmartDashboard.putNumber("Robot To Note FWD", robotToNote.get().getY()*39.37);
            SmartDashboard.putNumber("Robot To Note STR", robotToNote.get().getX()*39.37);
        }
       else{
            yawToNote = Optional.empty();
            pitchToNote = Optional.empty();
            robotToNote = Optional.empty();
        }
        SmartDashboard.putBoolean("Note is cut off", isNoteCutOffYaw());
        SmartDashboard.putBoolean("can see note", canSeeNote);
        SmartDashboard.putBoolean("Can see tag", canSeeTag);//cameras[0].canSeeTag(getSpeakerID()));
        SmartDashboard.putNumber("RPM to speaker", getShooterRPMToSpeaker());
        SmartDashboard.putNumber("Arm Pitch to Speaker", Math.toDegrees(getArmPitchToSpeaker()));
     }
    
}