// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxTheoreticalSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxSpeed = MaxTheoreticalSpeed * 0.8;
    private double MaxTheoreticalAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = MaxTheoreticalAngularRate * 0.8;
    private final double stickDeadband = 0.1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();






    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withHeadingPID(Constants.DriveConstants.aimkP, Constants.DriveConstants.aimkI, Constants.DriveConstants.aimkD);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final StateController stateController = new StateController(drivetrain, shooter, vision, intake);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

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

    public final Command extendHopper =
        new RunCommand(() -> intake.setMotors(-4.5), intake)
            .until(intake::isHopperExtended)
            .withTimeout(1.75)
            .andThen(new InstantCommand(() -> intake.setMotors(0), intake));

    public final Command retractHopper =
        new RunCommand(() -> intake.setMotors(4.5), intake)
            .until(intake::isHopperRetracted)
            .withTimeout(1.75)
            .andThen(new InstantCommand(() -> intake.setMotors(0), intake));

    public final InstantCommand resetPoseToHub = new InstantCommand(()->stateController.resetPoseToHub());
    public final InstantCommand resetPoseToLeftTrench = new InstantCommand(()->stateController.resetPoseToLeftTrench());
    public final InstantCommand resetPoseToRightTrench = new InstantCommand(()->stateController.resetPoseToRightTrench());
    public final InstantCommand resetPoseToVisionEstimate = new InstantCommand(()->stateController.resetPoseToVisionEstimate());
    
    public final Command travelOverBumpTimed = drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime);
    
    public RobotContainer() {
        configureBindings();
    }

    private double deadband(double stick) {
        return MathUtil.applyDeadband(stick, stickDeadband);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
 
         drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-deadband(driverController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed/2) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed/2) // Drive left with negative X (left)
                    .withRotationalRate(-deadband(driverController.getRightX()) * MaxAngularRate/2) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.y().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(0)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.b().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(-Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
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
            facingAngle.withVelocityX(-deadband(driverController.getLeftY()) * MaxSpeed)
                       .withVelocityY(-deadband(driverController.getLeftX()) * MaxSpeed)
                       .withTargetDirection(stateController.calculateAngleToAim())
        ));

        driverController.povUp().onTrue(resetPoseToHub);
        driverController.povLeft().onTrue(resetPoseToLeftTrench);
        driverController.povRight().onTrue(resetPoseToRightTrench);
        driverController.povDown().onTrue(resetPoseToVisionEstimate);

        // Reset field-centric heading: point the robot away from the driver and press Start
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Testing
        driverController.leftBumper().whileTrue(travelOverBumpTimed);

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
    }
    public void createAutos() {
        autoChooser.setDefaultOption("nothing", null);

//        autoChooser.addOption("C-SPL-RClimb", new PathPlannerAuto("C-SPL"));
        autoChooser.addOption("OL-SPL", new PathPlannerAuto("OL-SPL"));
        autoChooser.addOption("OR-SPL", new PathPlannerAuto("OR-SPL"));
        autoChooser.addOption("ORT-SPL-NZI-S", new PathPlannerAuto("ORT-SPL-NZI-S"));
        autoChooser.addOption("OLT-SPL-NZI-S", new PathPlannerAuto("OLT-SPL-NZI-S"));

        autoChooser.addOption("OL-NZ-I-SAZ", drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime).andThen(new PathPlannerAuto("OL-NZ-MI-SAZ")));
        autoChooser.addOption("OR-NZ-I-SAZ", drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime).andThen(new PathPlannerAuto("OR-NZ-MI-SAZ")));

        autoChooser.addOption("OL-NZ-AZ-S", new PathPlannerAuto("OL-NZ-AZ-S")
            .andThen(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime).andThen(new PathPlannerAuto("OL-NZ-AZ-I"))
            .andThen(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime)
            .andThen(drivetrain.runOnce(() -> drivetrain.resetTranslation(AllianceFlipUtil.apply(Constants.DriveConstants.OL_BUMP_AZ_BLUE.getTranslation()))))
            .andThen(new PathPlannerAuto("OL-NZ-AZ-S-Return")))));
        
            autoChooser.addOption("OR-NZ-AZ-S", new PathPlannerAuto("OR-NZ-AZ-S")
            .andThen(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime).andThen(new PathPlannerAuto("OR-NZ-AZ-I")))
            .andThen(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-2) 
                .withVelocityY(0) 
                .withRotationalRate(0)
            ).withTimeout(Constants.DriveConstants.bumpTravelTime)
            .andThen(drivetrain.runOnce(() -> drivetrain.resetTranslation(AllianceFlipUtil.apply(Constants.DriveConstants.OR_BUMP_AZ_BLUE.getTranslation()))))
            .andThen(new PathPlannerAuto("OR-NZ-AZ-S-Return"))));
        
        SmartDashboard.putData("autoChooser", autoChooser);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateControlsForAlliance(Alliance alliance) {
        stateController.updateAlliance(alliance);
    }
}
