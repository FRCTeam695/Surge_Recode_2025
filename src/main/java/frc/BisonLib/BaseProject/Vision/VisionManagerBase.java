package frc.BisonLib.BaseProject.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManagerBase extends SubsystemBase{

    protected final AprilTagCamera[] cameras;
    protected DoubleSupplier swerveHeadingSupplier;
    
    public VisionManagerBase(AprilTagCamera[] cameras){
        this.cameras = cameras;
    }

    public void setHeadingSupplier(DoubleSupplier headingSupplier){
        swerveHeadingSupplier = headingSupplier;
    }

    public VisionPosePacket[] getPoses(){
        VisionPosePacket[] poses = new VisionPosePacket[cameras.length];

        for(int i = 0; i < cameras.length; ++i){
            poses[i] = cameras[i].getLatestVisionUpdate(swerveHeadingSupplier.getAsDouble());
        }

        return poses;
    }
}
