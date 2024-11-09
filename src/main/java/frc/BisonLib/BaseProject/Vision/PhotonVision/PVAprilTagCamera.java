/* This file remains commented out until photonvision comes out with their 2025 vendordep */






// package frc.BisonLib.BaseProject.Vision.PhotonVision;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Transform3d;
// import frc.BisonLib.BaseProject.Vision.AprilTagCamera;
// import frc.BisonLib.BaseProject.Vision.VisionPosePacket;

// public class PVAprilTagCamera extends AprilTagCamera{

//     private PhotonCamera camera;
//     private PhotonPoseEstimator estimator;

//     private PhotonPipelineResult latestResult;

//     public PVAprilTagCamera(String name, double slope, Transform3d cameraToRobot){
//         super(name, slope);

//         latestResult = new PhotonPipelineResult();
//         camera = new PhotonCamera(name);
//         estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
//     }


//     public void loadLatestResult(){
//         latestResult = camera.getLatestResult();
//     }

//     public void setPriorityTagID(int id){}

//     @Override
//     public Double getYawToTag(int ID){
//         if(!latestResult.hasTargets()) return null;
//         else{
//             // Get a list of currently tracked targets.
//             List<PhotonTrackedTarget> targets = latestResult.getTargets();

//             for(PhotonTrackedTarget target : targets){
//                 if(target.getFiducialId() == ID) {
//                     return target.getYaw();
//                 };
//             }
//         }
//         return null;
//     }


//     //return target instead of pitch to get rid of uncessesary code + make it faster
//     public Double getPitchToTag(int ID){
//         if(!latestResult.hasTargets()) return null;
//         else{
//             // Get a list of currently tracked targets.
//             List<PhotonTrackedTarget> targets = latestResult.getTargets();

//             for(PhotonTrackedTarget target : targets){
//                 if(target.getFiducialId() == ID) {
//                     return target.getPitch();
//                 };
//             }
//         }
//         return null;
//     }

//     @Override
//     public Double getDistanceToTag(int ID){
//         if(!latestResult.hasTargets()) return null;
//         else{
//             // Get a list of currently tracked targets.
//             List<PhotonTrackedTarget> targets = latestResult.getTargets();

//             for(PhotonTrackedTarget target : targets){
//                 if(target.getFiducialId() == ID) {
//                     return target.getBestCameraToTarget().getTranslation().getNorm();
//                 };
//             }
//         }
//         return null;
//     }


//     public boolean canSeeTag(int ID){
//         var result = camera.getLatestResult();
//         if(!result.hasTargets()) return false;
//         else{
//             // Get a list of currently tracked targets.
//             List<PhotonTrackedTarget> targets = result.getTargets();

//             for(PhotonTrackedTarget target : targets){
//                 if(target.getFiducialId() == ID) {
//                     return true;
//                 };
//             }
//         }
//         return false;
//     }


//     @Override
//     public VisionPosePacket getLatestVisionUpdate(double yaw){
//         Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update();
//         if (poseEstimatorResult.isEmpty()) {
//             return new VisionPosePacket();
//         }

//         EstimatedRobotPose poseEstimate = poseEstimatorResult.get();
//         List<PhotonTrackedTarget> seenTags = poseEstimate.targetsUsed;

//         if (seenTags.size() >= 1 && seenTags.get(0).getPoseAmbiguity() > 0.2) {
//             return new VisionPosePacket(poseEstimate.timestampSeconds, poseEstimate.estimatedPose.toPose2d(), getAverageDistanceToTags(seenTags) * super.translation_slope, true);
//         }

        
//         return new VisionPosePacket();
//     }
//     /**
//      * Gets the mean distance to all tags visible when the pose was taken
//      * 
//      * @param tags A list of all the tracked tags
//      * @return The mean distance to all tracked tags
//      */
//     public double getAverageDistanceToTags(List<PhotonTrackedTarget> tags){
//         double sum = 0;
//         for(var tag : tags){
//             sum += tag.getBestCameraToTarget().getTranslation().getNorm();
//         }

//         return sum / tags.size();
//     }
// }