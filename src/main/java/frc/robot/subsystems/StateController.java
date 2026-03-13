package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.HubOffsetUtil;
import frc.robot.Util.TargetOffsetUtil;

public class StateController extends SubsystemBase{

    CommandSwerveDrivetrain drivetrain;
    ClimbSubsystem climbSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;
    IntakeSubsystem intakeSubsystem;
    HubOffsetUtil hubOffsetUtil;
    TargetOffsetUtil targetOffsetUtil;

    Pose2d currentPose;
    Pose2d targetPose;

    Field2d targetPoseField;

    enum Targets {
        HUB,
        ALLIANCE_ZONE,
        NEUTRAL_ZONE
    } 
    enum ZoneTargets {
        BLUE_ALLIANCE_1,
        BLUE_ALLIANCE_2,
        RED_ALLIANCE_1,
        RED_ALLIANCE_2
    }
    Targets currentTarget;
    ZoneTargets currentZoneTarget;
    Pose2d currentTargetPose;
    double currentAngle;
    double shotDampener;

    boolean intaking;
    boolean hopperExtended;
    boolean climbRotated;
    boolean climbExtended;
    boolean onBump;

    Alliance currentAlliance;
    Pose2d knownHubPose;
    Pose2d knownLeftTrenchPose;
    Pose2d knownRightTrenchPose;

    StructPublisher<Pose3d> frCameraPublisher;
    StructPublisher<Pose3d> blCameraPublisher;

    public StateController(CommandSwerveDrivetrain drive, ClimbSubsystem climb, ShooterSubsystem shooter,
                           VisionSubsystem vision, IntakeSubsystem intake) {
            drivetrain = drive;
            climbSubsystem = climb;
            shooterSubsystem = shooter;
            visionSubsystem = vision;
            intakeSubsystem = intake;
            hubOffsetUtil = new HubOffsetUtil(drive);
            targetOffsetUtil = new TargetOffsetUtil(drive);

            currentPose = drivetrain.getState().Pose;
            targetPose = new Pose2d();
            currentTarget = Targets.HUB;
            currentZoneTarget = ZoneTargets.BLUE_ALLIANCE_1;
            currentTargetPose = new Pose2d();
            shotDampener = Constants.ShooterConstants.shotRPMDampener;

            targetPoseField = new Field2d();
            targetPoseField.setRobotPose(targetPose);
            SmartDashboard.putData("TargetPoseVisualization", targetPoseField);
            currentAngle = 0;
            SmartDashboard.putNumber("CalculatedAngle", currentAngle);

            intaking = false;
            hopperExtended = false;
            climbRotated = false;
            climbExtended = false;
            onBump = false;

            currentAlliance = Alliance.Blue;
            knownHubPose = Constants.PoseConstants.knownBlueHubRobotPose;
            knownLeftTrenchPose = Constants.PoseConstants.knownBlueLeftTrenchRobotPose;
            knownRightTrenchPose = Constants.PoseConstants.knownBlueRightTrenchRobotPose;

            frCameraPublisher = NetworkTableInstance.getDefault().getStructTopic("frCameraPose", Pose3d.struct).publish(PubSubOption.sendAll(false));
            blCameraPublisher = NetworkTableInstance.getDefault().getStructTopic("blCameraPose", Pose3d.struct).publish(PubSubOption.sendAll(false));
   }

    // Shooter Methods
    public void setHoodAngle(double angle) {
        shooterSubsystem.setHoodAngle(angle);
    }
    public void setShooterSpeed(double speed) {
        shooterSubsystem.setDesiredSpeed(speed);
    }
    public void setIndexer(boolean load) {
        shooterSubsystem.setIndexer(load);
    }

    // Intake Methods
    public void setIntakePosition(double position) {
        intakeSubsystem.setPosition(position);
    }
    public void setIntakeWheels(boolean intake) {
        intakeSubsystem.intake(intake);
    }

    // Climb Methods
    public void setClimbAngle(double angle) {
        climbSubsystem.setDesiredAngle(angle);
    }
    public void setClimbExtension(double extension) {
        climbSubsystem.setDesiredPosition(extension);
    }

    // General Methods
    public void resetState() {
        setHoodAngle(0);
        setShooterSpeed(0);
        setIndexer(false);
        setIntakePosition(0);
        setIntakeWheels(false);
        setClimbAngle(0);
        setClimbExtension(0);
    }
    public void updateAlliance(Alliance alliance) {
        currentAlliance = alliance;
        if(alliance == Alliance.Blue) {
            knownHubPose = Constants.PoseConstants.knownBlueHubRobotPose;
            knownLeftTrenchPose = Constants.PoseConstants.knownBlueLeftTrenchRobotPose;
            knownRightTrenchPose = Constants.PoseConstants.knownBlueRightTrenchRobotPose;
        } else {
            knownHubPose = Constants.PoseConstants.knownRedHubRobotPose;
            knownLeftTrenchPose = Constants.PoseConstants.knownRedLeftTrenchRobotPose;
            knownRightTrenchPose = Constants.PoseConstants.knownRedRightTrenchRobotPose;
        }
    }
    public Rotation2d getForwardXAngle() {
        if(currentAlliance == Alliance.Blue) {
            return Constants.DriveConstants.blueForwardX;
        } else {
            return Constants.DriveConstants.redForwardX;
        }
    }
    public void addVisionMeasurement() {
        Optional<EstimatedRobotPose> frEstimate = visionSubsystem.getFREstimatedGlobalPose(currentPose);
            if (frEstimate.isPresent()) {

                EstimatedRobotPose estimate = frEstimate.get();

                if(estimate.estimatedPose != null){
                    frCameraPublisher.set((estimate.estimatedPose));
                    drivetrain.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    Timer.getFPGATimestamp()
                );
                }
            }

         Optional<EstimatedRobotPose> blEstimate = visionSubsystem.getBLEstimatedGlobalPose(currentPose);
            if (blEstimate.isPresent()) {

                EstimatedRobotPose estimate = blEstimate.get();

                if(estimate.estimatedPose != null){
                    frCameraPublisher.set((estimate.estimatedPose));
                    drivetrain.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    Timer.getFPGATimestamp()
                );
                }
            }
    }
    public void updateCurrentTarget() {
        if(currentAlliance == Alliance.Blue) {
            if(drivetrain.getState().Pose.getMeasureX().magnitude() < 4.63) {
                currentTarget = Targets.HUB;
            } else if (drivetrain.getState().Pose.getMeasureX().magnitude() < 11.94) {
                currentTarget = Targets.ALLIANCE_ZONE;
            } else {
                // either neutral zone or alliance zone
                currentTarget = Targets.ALLIANCE_ZONE;
            }
        } else {
            if(drivetrain.getState().Pose.getMeasureX().magnitude() > 11.94) {
                currentTarget = Targets.HUB;
            } else if (drivetrain.getState().Pose.getMeasureX().magnitude() > 4.63) {
                currentTarget = Targets.ALLIANCE_ZONE;
            } else {
                // either neutral zone or alliance zone
                currentTarget = Targets.ALLIANCE_ZONE;
            }
        }
    }
    public Pose2d predictFutureHubPose() {        
        return new Pose2d(hubOffsetUtil.predictFutureLocation(calculateFlywheelSpeed(), currentAlliance, shooterSubsystem.getCurrentHoodAngle()), new Rotation2d());
    }
    public Pose2d predictFutureTargetPose() {        
        return new Pose2d(targetOffsetUtil.predictFutureLocation(calculateFlywheelSpeed(), currentAlliance, shooterSubsystem.getCurrentHoodAngle()), new Rotation2d());
    }
    public Rotation2d calculateAngleToAim() {

        Rotation2d angle = new Rotation2d();
        Pose2d targetPose = new Pose2d();
        updateCurrentTarget();

        if(currentTarget.equals(Targets.HUB)) {
            targetPose = predictFutureHubPose();

            double dx = (AllianceFlipUtil.applyX(targetPose.getX()) - AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()));
            double dy = AllianceFlipUtil.applyY(targetPose.getY()) - AllianceFlipUtil.applyY(drivetrain.getState().Pose.getY());

            SmartDashboard.putNumber("CalculatedDx", dx);
            SmartDashboard.putNumber("CalculatedDy", dy);

            angle = (new Rotation2d(Math.atan2(dy,dx)));
        } else if(currentTarget.equals(Targets.ALLIANCE_ZONE)) {
            targetPose = predictFutureTargetPose();

            double dx = (AllianceFlipUtil.applyX(targetPose.getX()) - AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()));
            double dy = AllianceFlipUtil.applyY(targetPose.getY()) - AllianceFlipUtil.applyY(drivetrain.getState().Pose.getY());

            SmartDashboard.putNumber("CalculatedDx", dx);
            SmartDashboard.putNumber("CalculatedDy", dy);

            angle = (new Rotation2d(Math.atan2(dy,dx)));
        } else {
            // blah blah blah (neutral zone)
        }
        
        currentTargetPose = targetPose;
        if(currentAlliance == Alliance.Red) {
           angle.plus(Rotation2d.fromDegrees(180));
        }
        currentAngle = angle.getDegrees();
        return angle;
    }
    public Rotation2d simpleCalculateAngleToAim() {

        Pose2d targetPose = new Pose2d(new Translation2d(4.626,4.026), new Rotation2d(0));

        double dx = targetPose.getX() - drivetrain.getState().Pose.getX();
        double dy = targetPose.getY() - drivetrain.getState().Pose.getY();

        Rotation2d angle = new Rotation2d(Math.atan2(dy,dx));
        
        return angle.rotateBy(new Rotation2d(Units.degreesToRadians(180)));
    }
    public double calculateHoodAngle() {
        if(currentTarget.equals(Targets.HUB)){
  //          double distance = distanceToTarget();
  //          double distance = 0;

  //          double hoodAngle = Constants.ShooterConstants.maxHoodAngle - 
  //                              ((distance-Constants.ShooterConstants.minDistance)
  //                              /(Constants.ShooterConstants.maxDistance - Constants.ShooterConstants.minDistance))
  //                              * (Constants.ShooterConstants.maxHoodAngle - Constants.ShooterConstants.minHoodAngle);
        
  //          return hoodAngle;
        return Units.degreesToRadians(30);
        } 
        return Units.degreesToRadians(30);
    }
    public double calculateFlywheelSpeed() {

        double G = 9.81;
        double distance = distanceToTarget();
        double hoodAngle = calculateHoodAngle();

        double v = Math.sqrt((distance * G)/Math.sin(2*hoodAngle));
        SmartDashboard.putNumber("CalculatedDistanceToTarget", distance);
        return v; 
    }
    public double calculateShooterMotorSpeed(double velocity) {

            double[] velocities = Constants.ShooterConstants.correspondingExitVelocity;
            double[] outputs = Constants.ShooterConstants.correspondingMotorOutput;

            double v1 = 0;
            double v2 = 0;
            double m1 = 0;
            double m2 = 0;

            for(int i = 0; i < velocities.length-1; i++){
                if(velocities[i] <= velocity && velocity <= velocities[i+1]){
                    v1 = velocities[i];
                    v2 = velocities[i+1];
                    m1 = outputs[i];
                    m2 = outputs[i+1];

                    return m1 + ((velocity - v1) / (v2-v1)) * (m2 - m1);
                }
            }
        
        return 0;
    }
    public double distanceToTarget() {
        double distance = 0;
        Pose2d targetPose;

         if(currentTarget.equals(Targets.HUB)) {
            if(currentAlliance == Alliance.Blue){
                targetPose = Constants.PoseConstants.blueHub;
            } else {
                targetPose = Constants.PoseConstants.redHub;
            }

            double dx = (AllianceFlipUtil.applyX(targetPose.getX()) - AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()));
            double dy = AllianceFlipUtil.applyY(targetPose.getY()) - AllianceFlipUtil.applyY(drivetrain.getState().Pose.getY());

            distance = Math.hypot(dx, dy);
        } else {
            if(currentAlliance == Alliance.Blue){
            if(drivetrain.getState().Pose.getY() < 4.035) {
                targetPose = Constants.PoseConstants.blueAlliance1;
            } else{
                targetPose = Constants.PoseConstants.blueAlliance2;
            }
        } else {
            if(drivetrain.getState().Pose.getY() < 4.035) {
                targetPose = Constants.PoseConstants.redAlliance1;
            } else{
                targetPose = Constants.PoseConstants.redAlliance2;
            }
        }

            double dx = (AllianceFlipUtil.applyX(targetPose.getX()) - AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()));
            double dy = AllianceFlipUtil.applyY(targetPose.getY()) - AllianceFlipUtil.applyY(drivetrain.getState().Pose.getY());

            distance = Math.hypot(dx, dy);
        }

        return distance;
        
    }
    public void detectBump() {

        double robotRoll = drivetrain.getPigeon2().getRoll(true).getValueAsDouble();
        double robotPitch = drivetrain.getPigeon2().getPitch(true).getValueAsDouble();
        double currentRobotVelocityX = drivetrain.getState().Speeds.vxMetersPerSecond; // if > 0, moving toward +x
         
        if(!onBump && (Math.abs(robotRoll) > 12 || Math.abs(robotPitch) > 12)) {
            onBump = true;
        }
        else if(onBump && (Math.abs(robotRoll) < 1 && Math.abs(robotPitch) < 1)) {
            onBump = false;
            if(currentRobotVelocityX > 0) {
                drivetrain.getState().Pose.plus(new Transform2d(Constants.DriveConstants.bumpTravelDifference, 0.0, new Rotation2d()));
            } else {
                drivetrain.getState().Pose.minus(new Pose2d(Constants.DriveConstants.bumpTravelDifference, 0, new Rotation2d()));
            }
        }

    }
    public void periodic() {
   //     addVisionMeasurement();
   //     updateHoodAngle();
        targetPoseField.setRobotPose(currentTargetPose);
        
        SmartDashboard.putNumber("CalculatedAngle", currentAngle);
        SmartDashboard.putString("CurrentTarget", currentTarget.toString());
        SmartDashboard.putBoolean("Intaking", intaking);
        SmartDashboard.putBoolean("HopperExtended", hopperExtended);
        SmartDashboard.putBoolean("ClimbRotated", climbRotated);
        SmartDashboard.putBoolean("ClimbExtended", climbExtended);
        SmartDashboard.putString("CurrentTarget", currentTarget.toString());
        SmartDashboard.putString("CurrentZoneTarget", currentZoneTarget.toString());
        
    }

    // Robot Actions
    public void toggleIntake() {
        if(!intaking){
            setIntakeWheels(true);
            intaking = true;
        } else {
            setIntakeWheels(false);
            intaking = false;
        }
    }
    public void toggleHopper() {
        if(!hopperExtended){
            setIntakePosition(Constants.IntakeConstants.extendedPosition);
            hopperExtended = true;
        } else {
            setIntakePosition(Constants.IntakeConstants.retractedPosition);
            hopperExtended = false;
        }
    }
    public void shoot(boolean shoot) {
        if(shoot){
            setShooterSpeed(calculateShooterMotorSpeed(calculateFlywheelSpeed() * shotDampener));
  //          setHoodAngle(calculateHoodAngle());
            SmartDashboard.putNumber("CalculatedFlywheelOutput", calculateShooterMotorSpeed(calculateFlywheelSpeed() * shotDampener));
            SmartDashboard.putNumber("CalculatedFlywheelVelocityScaled", calculateFlywheelSpeed()*shotDampener);
        } else{
            setShooterSpeed(0);
        }
    }
    public void simpleShoot(boolean shoot) {
        if(hopperExtended) {
            setShooterSpeed(Constants.ShooterConstants.simpleShootingSpeedHopperExtended);
            setHoodAngle(Constants.ShooterConstants.simpleShootingAngle);
        } else {
            setShooterSpeed(Constants.ShooterConstants.simpleShootingSpeed);
        }
    }
    public void toggleClimbRotate() {
        if(!climbRotated) {
            setClimbAngle(Constants.ClimbConstants.climbReadyAngle);
            if(intaking) {
                toggleIntake();
            }
            if(hopperExtended){
                toggleHopper();
            }
            climbRotated = true;
        } else {
            setClimbAngle(Constants.ClimbConstants.climbRestingAngle);
            climbRotated = false;
        }
    }
    public void toggleClimbExtension() {
        if(!climbExtended) {
            setClimbExtension(Constants.ClimbConstants.climbExtendedDistance);
            climbExtended = true;
        } else {
            setClimbExtension(Constants.ClimbConstants.climbRetractedDistance);
        }
    }

    // Odometry Correction
    public void resetPoseToLeftTrench() {
        drivetrain.resetPose(knownLeftTrenchPose);
    }
    public void resetPoseToRightTrench() {
        drivetrain.resetPose(knownRightTrenchPose);
    }
    public void resetPoseToHub() {
        drivetrain.resetPose(knownHubPose);
    }

    public void resetPoseToVisionEstimate() {
        Pose2d poseEstimate = null;
        if(visionSubsystem.getFREstimatedGlobalPose(currentPose).isPresent()){
            poseEstimate = visionSubsystem.getFREstimatedGlobalPose(currentPose).get().estimatedPose.toPose2d();
        } else if(visionSubsystem.getBLEstimatedGlobalPose(currentPose).isPresent()) {
            poseEstimate = visionSubsystem.getBLEstimatedGlobalPose(currentPose).get().estimatedPose.toPose2d();
        }
        if(poseEstimate != null){
            drivetrain.resetPose(poseEstimate);
        }
    }
    
}
