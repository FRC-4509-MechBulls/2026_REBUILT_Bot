package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Util.HubOffsetUtil;
import frc.robot.Util.TargetOffsetUtil;
import frc.robot.sim.Field2d;
import frc.robot.sim.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

public class StateController {

    private final CommandSwerveDrivetrain drivetrain;
    private final ClimbSubsystem climbSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final HubOffsetUtil hubOffsetUtil;
    private final TargetOffsetUtil targetOffsetUtil;

    private Pose2d currentPose;
    private Pose2d targetPose;
    private final Field2d targetPoseField;

    private double currentAngle;

    private boolean intaking;
    private boolean hopperExtended;
    private boolean climbRotated;
    private boolean climbExtended;

    public StateController(CommandSwerveDrivetrain drive, ClimbSubsystem climb, ShooterSubsystem shooter,
                           VisionSubsystem vision, IntakeSubsystem intake) {
        drivetrain = drive;
        climbSubsystem = climb;
        shooterSubsystem = shooter;
        visionSubsystem = vision;
        intakeSubsystem = intake;

        hubOffsetUtil = new HubOffsetUtil(drive);
        targetOffsetUtil = new TargetOffsetUtil(drive);

        // FIX: getPose() instead of getState().Pose
        currentPose = drivetrain.getPose();
        targetPose = new Pose2d();
        currentAngle = 0;

        targetPoseField = new Field2d();
        targetPoseField.setRobotPose(targetPose);
        SmartDashboard.putData("TargetPoseVisualization", targetPoseField);
        SmartDashboard.putNumber("CalculatedAngle", currentAngle);

        intaking = false;
        hopperExtended = false;
        climbRotated = false;
        climbExtended = false;
    }

    // Subsystem control methods
    public void setHoodAngle(double angle) { shooterSubsystem.setHoodAngle(angle); }
    public void setShooterSpeed(double speed) { shooterSubsystem.setDesiredSpeed(speed); }
    public void setIndexer(boolean load) { shooterSubsystem.setIndexer(load); }
    public void setIntakePosition(double position) { intakeSubsystem.setPosition(position); }
    public void setIntakeWheels(boolean intake) { intakeSubsystem.intake(intake); }
    public void setClimbAngle(double angle) { climbSubsystem.setDesiredAngle(angle); }
    public void setClimbExtension(double extension) { climbSubsystem.setDesiredPosition(extension); }

    public void resetState() {
        setHoodAngle(0);
        setShooterSpeed(0);
        setIndexer(false);
        setIntakePosition(0);
        setIntakeWheels(false);
        setClimbAngle(0);
        setClimbExtension(0);
    }

    public void toggleIntake() {
        if (!intaking) {
            setIntakePosition(Constants.IntakeConstants.extendedPosition);
            setIntakeWheels(true);
            hopperExtended = true;
            intaking = true;
        } else {
            setIntakeWheels(false);
            intaking = false;
        }
    }

    public void toggleClimbRotate() {
        if (!climbRotated) {
            setClimbAngle(Constants.ClimbConstants.climbReadyAngle);
            climbRotated = true;
        } else {
            setClimbAngle(Constants.ClimbConstants.climbStowedAngle);
            climbRotated = false;
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("CalculatedAngle", currentAngle);
        targetPoseField.setRobotPose(currentPose);
    }

    // Simulation movement helper
    public void moveSim(double x, double y) {
        currentPose = new Pose2d(x, y, currentPose.getRotation());
    }

    // Getter for simulated field
    public Field2d getField2d() { return targetPoseField; }
}
