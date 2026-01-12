package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera1 = new PhotonCamera("Camera1");

    PhotonPoseEstimator poseEstimator;
    Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());

    public VisionSubsystem (){
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
        }catch (Exception e) {
            e.printStackTrace();
        }

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(camera1.isConnected()){
            List<PhotonPipelineResult> unreadResults = camera1.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> camera1Estimate = poseEstimator.update(latestResult);
                if(camera1Estimate.isPresent()) {
                    return camera1Estimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    
}
