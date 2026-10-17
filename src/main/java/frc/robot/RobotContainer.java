// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.HubShiftUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateController;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
    private double MaxTheoreticalSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = MaxTheoreticalSpeed;
    private double MaxTheoreticalAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
    private double MaxAngularRate = MaxTheoreticalAngularRate;
    private final double stickDeadband = 0.1;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.DriveConstants.driveAccelLimit / MaxSpeed);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.DriveConstants.driveAccelLimit / MaxSpeed);
    private double driveX = 0;
    private double driveY = 0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withHeadingPID(Constants.DriveConstants.aimkP, Constants.DriveConstants.aimkI, Constants.DriveConstants.aimkD);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    @Logged public final ShooterSubsystem shooter = new ShooterSubsystem();
    @Logged public final IntakeSubsystem intake = new IntakeSubsystem();
    @Logged public final StateController stateController = new StateController(drivetrain, shooter, intake);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final Map<String, Command> autos = new LinkedHashMap<>();
    private String selectedAuto = "Nothing - Hub";
    private final Map<Command, Pose2d> startingPoses = new HashMap<>();
    private Command lastPreviewedAuto = null;
    private Optional<Alliance> lastPreviewedAlliance = Optional.empty();

    private boolean wasTeleop = false;
    private boolean lastHubActive = false;
    private double lastShiftTimeLeft = 0;
    private double hubRumbleEndTime = 0;
    private double shiftWarningStartTime = -1;

    private final String[] moduleNames = {"FL", "FR", "BL", "BR"};
    private final Alert nothingAutoAlert = new Alert("Plain \"nothing\" auto selected: the robot won't know which way it's facing", AlertType.kWarning);
    private final Alert lowBatteryAlert = new Alert("Battery below 11.5 V", AlertType.kWarning);
    private final Alert drivetrainDisconnectedAlert = new Alert("Drivetrain device not responding", AlertType.kError);
    private final Debouncer drivetrainDisconnectedDebouncer = new Debouncer(0.5);

    public final InstantCommand resetRobot = new InstantCommand(()-> stateController.resetState());
    public final InstantCommand doShooting = new InstantCommand(()-> stateController.shoot(true));
    public final InstantCommand stopShooting = new InstantCommand(()-> stateController.shoot(false));
    public final InstantCommand doSimpleShooting = new InstantCommand(()-> stateController.simpleShoot(true));
    public final InstantCommand reverseShooter = new InstantCommand(()-> shooter.reverse());
    public final Command loadIndexer = Commands.waitUntil(shooter::isAtSpeed)
            .withTimeout(Constants.ShooterConstants.spinUpTimeout)
            .andThen(new InstantCommand(()->shooter.setIndexer(true)));
    public final InstantCommand stopIndexer = new InstantCommand(()->shooter.setIndexer(false));
    public final InstantCommand reverseIndexer = new InstantCommand(()->shooter.reverseIndexer());

    public final Command startIntake = new InstantCommand(()-> intake.intake(true), intake);
    public final Command stopIntake = new InstantCommand(()-> intake.intake(false), intake);

    public final Command extendHopper = extendHopperCommand();
    public final Command retractHopper = retractHopperCommand();

    public final InstantCommand resetPoseToHub = new InstantCommand(()->stateController.resetPoseToHub());
    public final InstantCommand resetPoseToLeftTrench = new InstantCommand(()->stateController.resetPoseToLeftTrench());
    public final InstantCommand resetPoseToRightTrench = new InstantCommand(()->stateController.resetPoseToRightTrench());

    public final Command travelOverBumpTimed = crossBumpCommand();

    public RobotContainer() {
        configureBindings();
    }

    private double deadband(double stick) {
        return MathUtil.applyDeadband(stick, stickDeadband);
    }

    private double rotationInput() {
        double rotation = deadband(driverController.getRightX());
        return -Math.copySign(rotation * rotation, rotation);
    }

    private Command extendHopperCommand() {
        return new RunCommand(() -> intake.setMotors(-4.5), intake)
            .until(intake::isHopperAtExtendedLimit)
            .withTimeout(1.75)
            .andThen(new InstantCommand(() -> {
                intake.setMotors(0);
                intake.markHopperExtended();
            }, intake));
    }

    private Command retractHopperCommand() {
        return new RunCommand(() -> intake.setMotors(4.5), intake)
            .until(intake::isHopperAtRetractedLimit)
            .withTimeout(1.75)
            .andThen(new InstantCommand(() -> {
                intake.setMotors(0);
                intake.markHopperRetracted();
            }, intake));
    }

    private Command crossBumpCommand() {
        return drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime);
    }

    private Command xLockCommand() {
        return drivetrain.applyRequest(() -> brake);
    }

    private Command oneButtonShotCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> stateController.simpleShoot(true), shooter),
                Commands.waitUntil(shooter::isAtSpeed).withTimeout(Constants.ShooterConstants.spinUpTimeout),
                Commands.runOnce(() -> shooter.setIndexer(true)),
                retractHopperCommand(),
                Commands.idle())
            .finallyDo(() -> stateController.shoot(false));
    }

    public void updateDriveInputs() {
        driveX = xLimiter.calculate(-deadband(driverController.getLeftY()));
        driveY = yLimiter.calculate(-deadband(driverController.getLeftX()));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

         drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationInput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveX * MaxSpeed/2) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed/2) // Drive left with negative X (left)
                    .withRotationalRate(rotationInput() * MaxAngularRate/2) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.y().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(driveX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(0)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(driveX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.b().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(driveX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(-Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(driveX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveY * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(Math.PI)) // Drive counterclockwise with negative X (left)
            )
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
//        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(()-> // While left trigger is held, drive while maintain a heading facing the target
            facingAngle.withVelocityX(driveX * MaxSpeed)
                       .withVelocityY(driveY * MaxSpeed)
                       .withTargetDirection(stateController.calculateAngleToAim())
        ));

        driverController.povUp().onTrue(resetPoseToHub);
        driverController.povLeft().onTrue(resetPoseToLeftTrench);
        driverController.povRight().onTrue(resetPoseToRightTrench);

        // Reset field-centric heading: point the robot away from the driver and press Start
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Testing
        driverController.leftBumper().whileTrue(travelOverBumpTimed);
        driverController.rightBumper().whileTrue(xLockCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.b().onTrue(resetRobot);
        operatorController.leftTrigger().whileTrue(reverseShooter); // spin the shooter backwards (no cameras, so no distance shot)
        operatorController.leftTrigger().whileFalse(stopShooting);
        operatorController.x().whileTrue(
            Commands.waitUntil(shooter::isAtSpeed).withTimeout(Constants.ShooterConstants.spinUpTimeout)
                .andThen(Commands.startEnd(()->shooter.setIndexer(true), ()->shooter.setIndexer(false), shooter)));
        operatorController.y().whileTrue(Commands.startEnd(()->shooter.reverseIndexer(), ()->shooter.setIndexer(false), shooter));
        operatorController.rightTrigger().whileTrue(doSimpleShooting);
        operatorController.rightTrigger().whileFalse(stopShooting);
        operatorController.leftBumper().onTrue(extendHopper);
        operatorController.rightBumper().onTrue(retractHopper);
        operatorController.start().onTrue(Commands.runOnce(()->intake.zeroHopper()).ignoringDisable(true));
        operatorController.povUp().whileTrue(oneButtonShotCommand());
        operatorController.povRight().and(RobotModeTriggers.disabled()).onTrue(Commands.runOnce(() -> cycleAuto(1)).ignoringDisable(true));
        operatorController.povLeft().and(RobotModeTriggers.disabled()).onTrue(Commands.runOnce(() -> cycleAuto(-1)).ignoringDisable(true));

        intake.setDefaultCommand(intake.run(()-> intake.controlIntake(
            deadband(operatorController.getLeftY()),
            operatorController.getHID().getAButton() ? Constants.IntakeConstants.intakeWheelSpeed : deadband(operatorController.getRightY()))));

        registerNamedCommands();
        createAutos();
    }
    public void registerNamedCommands() {
        NamedCommands.registerCommand("resetRobot", resetRobot);
        NamedCommands.registerCommand("doShooting", doShooting);
        NamedCommands.registerCommand("stopShooting", stopShooting);
        NamedCommands.registerCommand("doSimpleShooting", doSimpleShooting);
        NamedCommands.registerCommand("startIntake", startIntake);
        NamedCommands.registerCommand("stopIntake", stopIntake);
        NamedCommands.registerCommand("extendHopper", extendHopper);
        NamedCommands.registerCommand("retractHopper", retractHopper);
        NamedCommands.registerCommand("resetPoseToHub", resetPoseToHub);
        NamedCommands.registerCommand("resetPoseToLeftTrench", resetPoseToLeftTrench);
        NamedCommands.registerCommand("resetPoseToRightTrench", resetPoseToRightTrench);
        NamedCommands.registerCommand("loadIndexer", loadIndexer);
        NamedCommands.registerCommand("stopIndexer", stopIndexer);
        NamedCommands.registerCommand("reverseIndexer", reverseIndexer);
        NamedCommands.registerCommand("xLock", xLockCommand());
    }
    public void createAutos() {
        autos.put("Nothing - Hub", setStartingPose(Constants.PoseConstants.knownBlueHubRobotPose));
        autos.put("Defend - Left Trench", trenchDefenseAuto("Defend-LT"));
        autos.put("Defend - Right Trench", trenchDefenseAuto("Defend-RT"));
        autos.put("Defend - Left Bump", bumpDefenseAuto("Defend-LB", Constants.PoseConstants.leftBumpStartBlue));
        autos.put("Defend - Right Bump", bumpDefenseAuto("Defend-RB", Constants.PoseConstants.rightBumpStartBlue));
        autos.put("Nothing - Left Trench", setStartingPose(Constants.PoseConstants.knownBlueLeftTrenchRobotPose));
        autos.put("Nothing - Right Trench", setStartingPose(Constants.PoseConstants.knownBlueRightTrenchRobotPose));
        autos.put("nothing", null);

        for (Map.Entry<String, Command> auto : autos.entrySet()) {
            if (auto.getKey().equals(selectedAuto)) {
                autoChooser.setDefaultOption(auto.getKey(), auto.getValue());
            } else {
                autoChooser.addOption(auto.getKey(), auto.getValue());
            }
        }
        autoChooser.onChange(this::selectAuto);

        SmartDashboard.putData("autoChooser", autoChooser);
    }
    private void selectAuto(Command command) {
        for (Map.Entry<String, Command> auto : autos.entrySet()) {
            if (auto.getValue() == command) {
                selectedAuto = auto.getKey();
                return;
            }
        }
    }
    private void cycleAuto(int direction) {
        List<String> names = new ArrayList<>(autos.keySet());
        int index = names.indexOf(selectedAuto);
        selectedAuto = names.get(Math.floorMod(index + direction, names.size()));
    }
    public void publishSelectedAuto() {
        SmartDashboard.putString("SelectedAuto", selectedAuto);
    }
    private Command trenchDefenseAuto(String autoName) {
        PathPlannerAuto auto = new PathPlannerAuto(autoName);
        startingPoses.put(auto, auto.getStartingPose());
        return auto;
    }
    private Command bumpDefenseAuto(String autoName, Pose2d blueStart) {
        Command auto = drivetrain.runOnce(() -> drivetrain.resetPose(AllianceFlipUtil.apply(blueStart)))
            .andThen(crossBumpCommand())
            .andThen(new PathPlannerAuto(autoName));
        startingPoses.put(auto, blueStart);
        return auto;
    }
    private Command setStartingPose(Pose2d bluePose) {
        Pose2d intakeTowardOpponents = new Pose2d(bluePose.getTranslation(), new Rotation2d());
        Command command = drivetrain.runOnce(() -> drivetrain.resetPose(AllianceFlipUtil.apply(intakeTowardOpponents)));
        startingPoses.put(command, intakeTowardOpponents);
        return command;
    }
    public void previewStartingPose() {
        Command selected = getAutonomousCommand();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (selected == lastPreviewedAuto && alliance.equals(lastPreviewedAlliance)) {
            return;
        }
        lastPreviewedAuto = selected;
        lastPreviewedAlliance = alliance;

        Pose2d startingPose = startingPoses.get(selected);
        if (startingPose != null && !DriverStation.isFMSAttached()) {
            drivetrain.resetPose(AllianceFlipUtil.apply(startingPose));
        }
    }
    public Command getAutonomousCommand() {
        return autos.get(selectedAuto);
    }

    public void updateControlsForAlliance(Alliance alliance) {
        stateController.updateAlliance(alliance);
    }

    public void updateRumble() {
        double now = Timer.getFPGATimestamp();
        double hubRumble = 0;
        double readyRumble = 0;

        if (DriverStation.isTeleopEnabled()) {
            HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getShiftInfo();
            if (!wasTeleop) {
                lastHubActive = shiftInfo.active();
                lastShiftTimeLeft = shiftInfo.remainingTime();
                shiftWarningStartTime = -1;
            }
            if (shiftInfo.active() && !lastHubActive) {
                hubRumbleEndTime = now + 0.5;
            }
            if (shiftInfo.remainingTime() <= 3.0 && lastShiftTimeLeft > 3.0) {
                shiftWarningStartTime = now;
            }
            lastHubActive = shiftInfo.active();
            lastShiftTimeLeft = shiftInfo.remainingTime();

            double warningTime = now - shiftWarningStartTime;
            boolean warningPulse = shiftWarningStartTime >= 0
                && ((warningTime < 0.15) || (warningTime >= 0.3 && warningTime < 0.45));
            if (now < hubRumbleEndTime || warningPulse) {
                hubRumble = 1.0;
            }
            if (shooter.getDesiredSpeed() > 0 && shooter.isAtSpeed()) {
                readyRumble = 0.3;
            }
        }
        wasTeleop = DriverStation.isTeleopEnabled();

        driverController.getHID().setRumble(RumbleType.kBothRumble, hubRumble);
        operatorController.getHID().setRumble(RumbleType.kBothRumble, Math.max(hubRumble, readyRumble));
    }

    public void updateAlerts() {
        nothingAutoAlert.set(DriverStation.isDisabled() && getAutonomousCommand() == null);
        lowBatteryAlert.set(DriverStation.isDisabled() && RobotController.getBatteryVoltage() < 11.5);

        List<String> missingDevices = new ArrayList<>();
        for (int i = 0; i < moduleNames.length; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);
            if (!module.getDriveMotor().isConnected()) {
                missingDevices.add(moduleNames[i] + " drive");
            }
            if (!module.getSteerMotor().isConnected()) {
                missingDevices.add(moduleNames[i] + " steer");
            }
            if (!module.getEncoder().isConnected()) {
                missingDevices.add(moduleNames[i] + " CANcoder");
            }
        }
        if (!drivetrain.getPigeon2().isConnected()) {
            missingDevices.add("Pigeon");
        }
        if (!missingDevices.isEmpty()) {
            drivetrainDisconnectedAlert.setText("Drivetrain not responding: " + String.join(", ", missingDevices));
        }
        drivetrainDisconnectedAlert.set(drivetrainDisconnectedDebouncer.calculate(!missingDevices.isEmpty()));
    }
}
