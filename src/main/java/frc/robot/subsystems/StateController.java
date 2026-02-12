package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.HubOffsetUtil;
import frc.robot.Constants.DriveConstants;

public class StateController extends SubsystemBase{

    CommandSwerveDrivetrain drivetrain;
    ClimbSubsystem climbSubsystem;
    ShooterSubsystem shooterSubsystem;
    VisionSubsystem visionSubsystem;
    IntakeSubsystem intakeSubsystem;
    HubOffsetUtil hubOffsetUtil;

    Pose2d currentPose;
    Pose2d targetPose;

    Field2d targetPoseField;

    enum Targets {
        HUB,
        ALLIANCE_ZONE,
        NEUTRAL_ZONE
    } 
    Targets currentTarget;
    Pose2d currentTargetPose;
    double currentAngle;

    Alliance currentAlliance;

    public StateController(CommandSwerveDrivetrain drive, ClimbSubsystem climb, ShooterSubsystem shooter,
                           VisionSubsystem vision, IntakeSubsystem intake) {
            drivetrain = drive;
            climbSubsystem = climb;
            shooterSubsystem = shooter;
            visionSubsystem = vision;
            intakeSubsystem = intake;
            hubOffsetUtil = new HubOffsetUtil(drive);

            currentPose = drivetrain.getState().Pose;
            targetPose = new Pose2d();
            currentTarget = Targets.HUB;
            currentTargetPose = new Pose2d();

            targetPoseField = new Field2d();
            targetPoseField.setRobotPose(targetPose);
            SmartDashboard.putData("TargetPoseVisualization", targetPoseField);

            currentAngle = 0;
            SmartDashboard.putNumber("CalculatedAngle", currentAngle);

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
        if(visionSubsystem.getEstimatedGlobalPose(currentPose).isPresent()){
            drivetrain.addVisionMeasurement(visionSubsystem.getEstimatedGlobalPose(currentPose).get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
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
    public Pose2d predictFuturePose() {        
        return new Pose2d(hubOffsetUtil.predictFutureLocation(calculateFlywheelSpeed(), currentAlliance), new Rotation2d());
    }
    public Rotation2d calculateAngleToAim() {

        Rotation2d angle = new Rotation2d();
        Pose2d targetPose = new Pose2d();
        //updateCurrentTarget();

        if(currentTarget.equals(Targets.HUB)) {
            targetPose = predictFuturePose();

            double dx = (AllianceFlipUtil.applyX(targetPose.getX()) - AllianceFlipUtil.applyX(drivetrain.getState().Pose.getX()));
            double dy = AllianceFlipUtil.applyY(targetPose.getY()) - AllianceFlipUtil.applyY(drivetrain.getState().Pose.getY());

            SmartDashboard.putNumber("CalculatedDx", dx);
            SmartDashboard.putNumber("CalculatedDy", dy);

            angle = (new Rotation2d(Math.atan2(dy,dx)));
        } else if(currentTarget.equals(Targets.ALLIANCE_ZONE)) {
            // blah blah blah
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

        // take distance and hood angle and make up a flywheel speed

        return 10;
    }
    public void updateHoodAngle() {
        if(currentTarget.equals(Targets.ALLIANCE_ZONE) || currentTarget.equals(Targets.NEUTRAL_ZONE)) {
            setHoodAngle(Constants.ShooterConstants.hoodAngle1);
        } else {
            setHoodAngle(Constants.ShooterConstants.hoodAngle2);
        }
    }
    public void periodic() {
        addVisionMeasurement();
        updateHoodAngle();
        targetPoseField.setRobotPose(currentTargetPose);
        
        SmartDashboard.putNumber("CalculatedAngle", currentAngle);
    }

    public void intake(boolean intake) {
        if(intake){
            setIntakePosition(Constants.IntakeConstants.extendedPosition);
            setIntakeWheels(true);
        } else {
            setIntakePosition(Constants.IntakeConstants.retractedPosition);
            setIntakeWheels(false);
        }
    }
    public void shoot(boolean shoot) {
        if(shoot){
            setShooterSpeed(calculateFlywheelSpeed());
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setIndexer(true);
        } else{
            setShooterSpeed(0);
            setIndexer(false);
        }
    }

}
