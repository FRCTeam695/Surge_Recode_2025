package frc.BisonLib.BaseProject.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManagerBase extends SubsystemBase{

    protected final AprilTagCamera[] cameras;
    protected final DoubleSupplier swerveHeadingSupplier;
    
    public VisionManagerBase(AprilTagCamera[] cameras, DoubleSupplier swerveHeadingSupplier){
        this.cameras = cameras;
        this.swerveHeadingSupplier = swerveHeadingSupplier;
    }

    public VisionPosePacket[] getPoses(){
        VisionPosePacket[] poses = new VisionPosePacket[cameras.length];

        for(int i = 0; i < cameras.length; ++i){
            poses[i] = cameras[i].getLatestVisionUpdate(swerveHeadingSupplier.getAsDouble());
        }

        return poses;
    }
}
