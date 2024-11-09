package frc.BisonLib.BaseProject.Vision.Limelight;

import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
import frc.BisonLib.BaseProject.Vision.VisionPosePacket;


/*
 * LLAprilTagCamera is helps with dealing with limelights and april tags
 * Each Limelight that is looking at april tags should be its own LLAprilTagCamera object
 */
public class LLAprilTagCamera extends AprilTagCamera{

    /**
     * @param name The name of the camera
     * @param slope Distance To Tag * slope = Standard Deviation (The further away we are the less we trust the tag --> higher standard deviation)
     */
    public LLAprilTagCamera(String name, double slope){
        super(name, slope);
    }

    /**
     * Gets the latest pose recorded by MegaTag2
     * Docs - https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
     * 
     * @param yaw The yaw to tell MegaTag2 that we are facing
     * 
     * @return VisionPacket that holds all the relevent information about the pose
     */
    @Override
    public VisionPosePacket getLatestVisionUpdate(double yaw){
        LimelightHelpers.SetRobotOrientation(super.kCamName, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(super.kCamName);
        return new VisionPosePacket(mt2.timestampSeconds, mt2.pose, mt2.avgTagDist * translation_slope, !(mt2.tagCount == 0));
    }  


    /**
     * Returns the yaw from the camera to the specified tag
     * Limelight NetworkTables docs - https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
     * 
     * @param ID the ID of the tag we want to see
     * 
     * @return The TX to the tag
     */
    @Override
    public Double getYawToTag(int ID){
        LimelightHelpers.setPriorityTagID(super.kCamName, ID);
        if(canSeeTargets()) return LimelightHelpers.getTX(super.kCamName);
        else return null;
    }


    @Override
    public Double getDistanceToTag(int ID){
        return 0.;
    }

    public void setPriorityTagID(int ID){
        LimelightHelpers.setPriorityTagID(super.kCamName, ID);
    }
    public Double getPitchToTag(int ID){
        LimelightHelpers.setPriorityTagID(super.kCamName, ID);
        if(canSeeTargets()) return LimelightHelpers.getTY(super.kCamName);
        else return null;
    }

    /**
     * Checks if the camera has any tags in view
     * Limelight NetworkTables docs - https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
     * 
     * @return If the camera can see any tags
     */
    public boolean canSeeTargets(){
        return LimelightHelpers.getTV(super.kCamName);
    }

    public boolean canSeeTag(int ID){
        return false;
    }

    public void loadLatestResult(){
        
    }
}
