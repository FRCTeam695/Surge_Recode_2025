package frc.BisonLib.BaseProject.Vision.Limelight;

public class LLIntakeCamera {

    public final String name;

    public LLIntakeCamera(String name){
        this.name = name;
    }

    public double getYawToTarget(){
        return LimelightHelpers.getTX(name);
    }

    public double getPitchToTarget(){
        return LimelightHelpers.getTY(name);
    }

    public boolean canSeeTarget(){
        return LimelightHelpers.getTV(name);
    }
}
