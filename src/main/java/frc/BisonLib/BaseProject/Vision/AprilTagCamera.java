package frc.BisonLib.BaseProject.Vision;

public abstract class AprilTagCamera {
    public String kCamName;
    public final double translation_slope;

    /**
     * @param name The name of the camera
     * @param slope Distance To Tag * slope = Standard Deviation (The further away we are the less we trust the tag --> higher standard deviation)
     */
    public AprilTagCamera(String name, double slope){
        this.kCamName = name;
        this.translation_slope = slope;
    }

    /**
     * Gets all relevant info about the latest robot pose read frmo vision
     * 
     * @param yaw The current robot heading
     * @return VisionPacket containing info about the robot pose
     */
    public abstract VisionPosePacket getLatestVisionUpdate(double yaw);

    /**
     * Gets the yaw to an april tag
     * 
     * @param ID The ID of the tag to get the yaw from
     * @return The yaw from camera - tag
     */
    public abstract Double getYawToTag(int ID);

    public abstract Double getPitchToTag(int ID);

    public abstract Double getDistanceToTag(int ID);

    public abstract void setPriorityTagID(int ID);


    public abstract boolean canSeeTag(int ID);

    public abstract void loadLatestResult();

}
