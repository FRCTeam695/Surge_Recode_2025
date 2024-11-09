package frc.BisonLib.BaseProject.Vision;

import edu.wpi.first.math.geometry.Pose2d;

/*
 * VisionPacket is holds all the important information pertaining to a pose
 */
public class VisionPosePacket {
    
    public double timestamp;
    public Pose2d pose;
    public double stdDev;
    public boolean isValidPose;

    /**
     * @param timestamp The time when the pose was taken
     * @param pose The pose from vision
     * @param stdDev The standard deviation calculated from vision (how accurate it thinks the pose is)
     * @param isValidPose If the pose is valid or not (If the camera doesn't see any tags it returns the last recorded pose)
     */
    public VisionPosePacket(double timestamp, Pose2d pose, double stdDev, boolean isValidPose){
        this.timestamp = timestamp;
        this.pose = pose;
        this.stdDev = stdDev;
        this.isValidPose = isValidPose;
    }

    public VisionPosePacket(){
        this.isValidPose = false;
    }
}
