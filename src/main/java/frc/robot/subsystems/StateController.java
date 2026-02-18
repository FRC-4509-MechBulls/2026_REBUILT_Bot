package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    boolean intaking;
    boolean hopperExtended;
    boolean climbRotated;
    boolean climbExtended;
    boolean onBump;

    Alliance currentAlliance;

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
    }
    public void addVisionMeasurement() {
        if(visionSubsystem.getFLEstimatedGlobalPose(currentPose).isPresent()){
            drivetrain.addVisionMeasurement(visionSubsystem.getFLEstimatedGlobalPose(currentPose).get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
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
        return new Pose2d(hubOffsetUtil.predictFutureLocation(calculateFlywheelSpeed(), currentAlliance), new Rotation2d());
    }
    public Pose2d predictFutureTargetPose() {        
        return new Pose2d(targetOffsetUtil.predictFutureLocation(calculateFlywheelSpeed(), currentAlliance), new Rotation2d());
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
    public double calculateFlywheelSpeed() {

        // use a linear regression model for experiment data points that graph horizontal distance (x)
        // from hub vs the required motor speed to consistently score fuel

        return 10; 
    }
    public double calculateFuelExitVelocity() {
        // should be some fraction of the flywheel speed

        return 10;
    }
    public void updateHoodAngle() {
        if(currentTarget.equals(Targets.ALLIANCE_ZONE) || currentTarget.equals(Targets.NEUTRAL_ZONE)) {
            setHoodAngle(Constants.ShooterConstants.hoodAngle1);
        } else {
            setHoodAngle(Constants.ShooterConstants.hoodAngle2);
        }
    }
    public void detectBump() {

        double robotRoll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
        double robotPitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
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
        addVisionMeasurement();
        updateHoodAngle();
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
            setIntakePosition(Constants.IntakeConstants.extendedPosition);
            setIntakeWheels(true);
            hopperExtended = true;
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
            setShooterSpeed(calculateFlywheelSpeed());
            try {
                Thread.sleep(Constants.ShooterConstants.windUpTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setIndexer(true);
        } else{
            setShooterSpeed(0);
            setIndexer(false);
        }
    }
    public void simpleShoot(boolean shoot) {
        setShooterSpeed(Constants.ShooterConstants.simpleShootingSpeed);
        try {
            Thread.sleep(Constants.ShooterConstants.windUpTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setIndexer(true);
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

}
