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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera frontRightCamera = new PhotonCamera("FrontRightCamera");
    PhotonCamera backLeftCamera = new PhotonCamera("BackLeftCamera");

    PhotonPoseEstimator frontRightPoseEstimator;
    PhotonPoseEstimator backLeftPoseEstimator;

    Transform3d robotToFrontRightCamera = Constants.VisionConstants.robotToFrontRightCamera;
    Transform3d robotToBackLeftCamera = Constants.VisionConstants.robotToBackLeftCamera;

    Field2d frontRightEstimatePose;
    Field2d backLeftEstimatePose;

    // Testing camera transforms
    // StructPublisher<Pose3d> frCameraPublisher = NetworkTableInstance.getDefault().getStructTopic("frCameraPose", Pose3d.struct).publish(PubSubOption.sendAll(false));
    // StructPublisher<Pose3d> blCameraPublisher = NetworkTableInstance.getDefault().getStructTopic("blCameraPose", Pose3d.struct).publish(PubSubOption.sendAll(false));

    public VisionSubsystem (){
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
        }catch (Exception e) {
            e.printStackTrace();
        }

        frontRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontRightCamera);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        backLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackLeftCamera);
        backLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        frontRightEstimatePose = new Field2d();
        backLeftEstimatePose = new Field2d();

        SmartDashboard.putData(frontRightEstimatePose);
        SmartDashboard.putData(backLeftEstimatePose);
       
    }
    public Optional<EstimatedRobotPose> getFREstimatedGlobalPose(Pose2d prevEstimatedRobotPose) { 
        if(frontRightCamera.isConnected()){
            List<PhotonPipelineResult> unreadResults = frontRightCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) {
                PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
                Optional<EstimatedRobotPose> frontRightCameraEstimate = frontRightPoseEstimator.estimateCoprocMultiTagPose(latestResult);
                if(frontRightCameraEstimate.isPresent()) {
                    frontRightEstimatePose.setRobotPose(frontRightCameraEstimate.get().estimatedPose.toPose2d());
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
                    backLeftEstimatePose.setRobotPose(backLeftCameraEstimate.get().estimatedPose.toPose2d());
                    return backLeftCameraEstimate;
                }
            }
            return Optional.empty();
        }
        return Optional.empty();
    }

    public void periodic() {
        // frCameraPublisher.set(new Pose3d().plus(Constants.VisionConstants.robotToFrontRightCamera));
        // blCameraPublisher.set(new Pose3d().plus(Constants.VisionConstants.robotToBackLeftCamera));
    }
}
