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
    PhotonCamera backRightCamera = new PhotonCamera("backRightCamera");


    PhotonPoseEstimator frontLeftPoseEstimator;
    PhotonPoseEstimator frontRightPoseEstimator;
    PhotonPoseEstimator backLeftPoseEstimator;
    PhotonPoseEstimator backRightPoseEstimator;
  //aaaa

    SwerveDrivePoseEstimator masterPoseEstimator;

    Transform3d robotToFrontLeftCamera = new Transform3d(new Translation3d(), new Rotation3d());
    Transform3d robotToFrontRightCamera = new Transform3d(new Translation3d(), new Rotation3d());
    Transform3d robotToBackLeftCamera = new Transform3d(new Translation3d(), new Rotation3d());
    Transform3d robotToBackRightCamera = new Transform3d(new Translation3d(), new Rotation3d());

    public VisionSubsystem (){
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
        }catch (Exception e) {
            e.printStackTrace();
        }

        frontLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontLeftCamera);
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        frontRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontRightCamera);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        backLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackLeftCamera);
        backLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        backRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackRightCamera);
        backRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  
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

    public Optional<EstimatedRobotPose> getFREstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(frontRightCamera.isConnected()){
            List<PhotonPipelineResult> unreadResults = frontRightCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> frontRightCameraEstimate = frontRightPoseEstimator.estimateCoprocMultiTagPose(latestResult);
                if(frontRightCameraEstimate.isPresent()) {
                    return frontRightCameraEstimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    public Optional<EstimatedRobotPose> getBLEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(backLeftCamera.isConnected()){
            List<PhotonPipelineResult> unreadResults = backLeftCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> backLeftCameraEstimate = backLeftPoseEstimator.estimateCoprocMultiTagPose(latestResult);
                if(backLeftCameraEstimate.isPresent()) {
                    return backLeftCameraEstimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    public Optional<EstimatedRobotPose> getBREstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         
        if(backRightCamera.isConnected()){
            List<PhotonPipelineResult> unreadResults = backRightCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> backRightCameraEstimate = backRightPoseEstimator.estimateCoprocMultiTagPose(latestResult);
                if(backRightCameraEstimate.isPresent()) {
                    return backRightCameraEstimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    


    public void periodic() {
        
    }
}
