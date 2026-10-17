package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.HubOffsetUtil;
import frc.robot.Util.TargetOffsetUtil;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class StateController extends SubsystemBase{

    CommandSwerveDrivetrain drivetrain;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    HubOffsetUtil hubOffsetUtil;
    TargetOffsetUtil targetOffsetUtil;

    Field2d targetPoseField;
    InterpolatingDoubleTreeMap shotMap;

    enum Targets {
        HUB,
        ALLIANCE_ZONE,
        NEUTRAL_ZONE
    }
    @Logged Targets currentTarget;
    Pose2d currentTargetPose;
    @Logged double currentAngle;

    @Logged boolean intaking;
    @Logged boolean hopperExtended;

    Alliance currentAlliance;
    Pose2d knownHubPose;
    Pose2d knownLeftTrenchPose;
    Pose2d knownRightTrenchPose;

    public StateController(CommandSwerveDrivetrain drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
            drivetrain = drive;
            shooterSubsystem = shooter;
            intakeSubsystem = intake;
            hubOffsetUtil = new HubOffsetUtil(drive);
            targetOffsetUtil = new TargetOffsetUtil(drive);

            currentTarget = Targets.HUB;
            currentTargetPose = new Pose2d();

            shotMap = new InterpolatingDoubleTreeMap();
            for(int i = 0; i < Constants.ShooterConstants.shotDistances.length; i++) {
                shotMap.put(Constants.ShooterConstants.shotDistances[i], Constants.ShooterConstants.shotOutputs[i]);
            }
            SmartDashboard.setDefaultBoolean("ShotTuningMode", false);
            SmartDashboard.setDefaultNumber("ShotTuningOutput", Constants.ShooterConstants.simpleShootingSpeed);

            targetPoseField = new Field2d();
            targetPoseField.setRobotPose(currentTargetPose);
            SmartDashboard.putData("TargetPoseVisualization", targetPoseField);
            currentAngle = 0;
            SmartDashboard.putNumber("CalculatedAngle", currentAngle);

            intaking = false;
            hopperExtended = false;

            currentAlliance = Alliance.Blue;
            knownHubPose = Constants.PoseConstants.knownBlueHubRobotPose;
            knownLeftTrenchPose = Constants.PoseConstants.knownBlueLeftTrenchRobotPose;
            knownRightTrenchPose = Constants.PoseConstants.knownBlueRightTrenchRobotPose;
   }

    // Shooter Methods
    public void setShooterSpeed(double speed) {
        shooterSubsystem.setDesiredSpeed(speed);
    }
    public void setIndexer(boolean load) {
        shooterSubsystem.setIndexer(load);
    }

    // Intake Methods
    public void setIntakeWheels(boolean intake) {
        intakeSubsystem.intake(intake);
    }

    // General Methods
    public void resetState() {
        setShooterSpeed(0);
        setIndexer(false);
        intakeSubsystem.setMotors(0);
        setIntakeWheels(false);
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
        currentAngle = angle.getDegrees();
        return angle;
    }
    public double calculateHoodAngle() {
        return Units.degreesToRadians(Constants.ShooterConstants.scoringHoodAngle);
    }
    public double calculateFlywheelSpeed() {

        double G = 9.81;
        double distance = distanceToTarget();
        double hoodAngle = calculateHoodAngle();

        double v = Math.sqrt((distance * G)/Math.sin(2*hoodAngle));
        return v;
    }
    @Logged
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
    public boolean isShotTuningMode() {
        return SmartDashboard.getBoolean("ShotTuningMode", false);
    }
    @Logged
    public double getShotOutput() {
        if(isShotTuningMode()) {
            return SmartDashboard.getNumber("ShotTuningOutput", Constants.ShooterConstants.simpleShootingSpeed);
        }
        return shotMap.get(distanceToTarget());
    }
    public void periodic() {
        targetPoseField.setRobotPose(currentTargetPose);
        updateCurrentTarget();
        hopperExtended = intakeSubsystem.isHopperExtended();
        intaking = intakeSubsystem.isIntaking();

        SmartDashboard.putNumber("CalculatedDistanceToTarget", distanceToTarget());
        SmartDashboard.putNumber("ShotTableOutput", shotMap.get(distanceToTarget()));
        SmartDashboard.putNumber("CalculatedAngle", currentAngle);
        SmartDashboard.putString("CurrentTarget", currentTarget.toString());
        SmartDashboard.putBoolean("Intaking", intaking);
        SmartDashboard.putBoolean("HopperExtended", hopperExtended);

    }

    // Robot Actions
    public void shoot(boolean shoot) {
        if(shoot){
            setShooterSpeed(getShotOutput());
        } else{
            setShooterSpeed(0);
            setIndexer(false);
        }
    }
    public void simpleShoot(boolean shoot) {
        if(isShotTuningMode()) {
            setShooterSpeed(getShotOutput());
        } else if(hopperExtended) {
            setShooterSpeed(Constants.ShooterConstants.simpleShootingSpeedHopperExtended);
        } else {
            setShooterSpeed(Constants.ShooterConstants.simpleShootingSpeed);
        }
    }


    // Odometry Correction
    public void resetPoseToLeftTrench() {
        drivetrain.resetTranslation(knownLeftTrenchPose.getTranslation());
    }
    public void resetPoseToRightTrench() {
        drivetrain.resetTranslation(knownRightTrenchPose.getTranslation());
    }
    public void resetPoseToHub() {
        drivetrain.resetTranslation(knownHubPose.getTranslation());
    }

}
