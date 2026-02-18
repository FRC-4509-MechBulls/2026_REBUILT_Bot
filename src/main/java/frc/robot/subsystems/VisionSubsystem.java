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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera frontLeftCamera = new PhotonCamera("frontLeftCamera");
    PhotonCamera frontRightCamera = new PhotonCamera("frontRightCamera");
    PhotonCamera backLeftCamera = new PhotonCamera("backLeftCamera");

    PhotonPoseEstimator frontLeftPoseEstimator;
    PhotonPoseEstimator frontRightPoseEstimator;
    PhotonPoseEstimator backLeftPoseEstimator;

    SwerveDrivePoseEstimator masterPoseEstimator;

    Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());

    public VisionSubsystem (){
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
        }catch (Exception e) {
            e.printStackTrace();
        }

        frontLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        frontRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        backLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        backLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    }

    public Optional<EstimatedRobotPose> getFLEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(frontLeftCamera.isConnected()){
            List<PhotonPipelineResult> unreadResults = frontLeftCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> frontLeftCameraEstimate = frontLeftPoseEstimator.estimateCoprocMultiTagPose(latestResult);
                if(frontLeftCameraEstimate.isPresent()) {
                    return frontLeftCameraEstimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    public void periodic() {
        
    }
}
