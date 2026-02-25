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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
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

    /* Setting up bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed*0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withHeadingPID(Constants.DriveConstants.aimkP, Constants.DriveConstants.aimkI, Constants.DriveConstants.aimkD);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();
    public final StateController stateController = new StateController(drivetrain, climb, shooter, vision, intake);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final InstantCommand resetRobot = new InstantCommand(()-> stateController.resetState()); 
    public final InstantCommand doShooting = new InstantCommand(()-> stateController.shoot(true));
    public final InstantCommand stopShooting = new InstantCommand(()-> stateController.shoot(false));    
    public final InstantCommand doSimpleShooting = new InstantCommand(()-> stateController.simpleShoot(true));
    public final InstantCommand toggleIntake = new InstantCommand(()-> stateController.toggleIntake());   
    public final InstantCommand toggleHopper = new InstantCommand(()-> stateController.toggleHopper());
    public final InstantCommand toggleClimbRotate = new InstantCommand(()-> stateController.toggleClimbRotate());
    public final InstantCommand toggleClimbExtension = new InstantCommand(()-> stateController.toggleClimbExtension());   
    public final InstantCommand resetPoseToHub = new InstantCommand(()->stateController.resetPoseToHub());
    public final InstantCommand resetPoseToLeftTrench = new InstantCommand(()->stateController.resetPoseToLeftTrench());
    public final InstantCommand resetPoseToRightTrench = new InstantCommand(()->stateController.resetPoseToRightTrench());
    public final InstantCommand resetPoseToVisionEstimate = new InstantCommand(()->stateController.resetPoseToVisionEstimate());
    
    public final SequentialCommandGroup climbCommandGroup = new SequentialCommandGroup(
                                new InstantCommand(()-> stateController.toggleClimbExtension())
                                .andThen(new WaitCommand(1))
                                .andThen(new InstantCommand(()-> stateController.toggleClimbRotate()))
                                .andThen(new WaitCommand(2)
                                .andThen(drivetrain.applyRequest(() ->
                                            drive.withVelocityX(1) // Drive forward with negative Y (forward)
                                                .withVelocityY(0) // Drive left with negative X (left)
                                                .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                                            )).withTimeout(2)
                                .andThen(new InstantCommand(()-> stateController.toggleClimbExtension()))));

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.y().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(0)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.b().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withTargetDirection(new Rotation2d(-Math.PI/2)) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
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
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
//        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // maybe remove this?

        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(()-> // While left trigger is held, drive while maintain a heading facing the target
            facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                       .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                       .withTargetDirection(stateController.calculateAngleToAim())
        ));

        driverController.povUp().onTrue(resetPoseToHub);
        driverController.povLeft().onTrue(resetPoseToLeftTrench);
        driverController.povRight().onTrue(resetPoseToRightTrench);
        driverController.povDown().onTrue(resetPoseToVisionEstimate);
        driverController.rightBumper().onTrue(toggleClimbRotate);
        driverController.rightTrigger().onTrue(toggleClimbExtension);

        drivetrain.registerTelemetry(logger::telemeterize);
        
        operatorController.b().onTrue(resetRobot);
        operatorController.leftTrigger().onTrue(doShooting);
        operatorController.leftTrigger().onFalse(stopShooting);
        operatorController.leftBumper().onTrue(doSimpleShooting);
        operatorController.leftBumper().onFalse(stopShooting);
        operatorController.a().onTrue(toggleIntake);
        operatorController.x().onTrue(toggleHopper);
        
        registerNamedCommands();
        createAutos();
    }
    public void registerNamedCommands() {
        NamedCommands.registerCommand("resetRobot", resetRobot);
        NamedCommands.registerCommand("doShooting", doShooting);
        NamedCommands.registerCommand("stopShooting", stopShooting);
        NamedCommands.registerCommand("doSimpleShooting", doSimpleShooting);
        NamedCommands.registerCommand("toggleIntake", toggleIntake);
        NamedCommands.registerCommand("toggleHopper", toggleHopper);
        NamedCommands.registerCommand("climbCommandGroup", climbCommandGroup);
        NamedCommands.registerCommand("resetPoseToHub", resetPoseToHub);
        NamedCommands.registerCommand("resetPoseToLeftTrench", resetPoseToLeftTrench);
        NamedCommands.registerCommand("resetPoseToRightTrench", resetPoseToRightTrench);
    }
    public void createAutos() {
        autoChooser.setDefaultOption("nothing", null);

        autoChooser.addOption("C-SPL-CClimb", new PathPlannerAuto("C-SPL-CClimb"));
        autoChooser.addOption("C-SPL-LClimb", new PathPlannerAuto("C-SPL-LClimb"));
        autoChooser.addOption("C-SPL-RClimb", new PathPlannerAuto("C-SPL-RClimb"));
        autoChooser.addOption("OL-SPL-LClimb", new PathPlannerAuto("OL-SPL-LClimb"));
        autoChooser.addOption("OL-SPL-SDEP-LClimb", new PathPlannerAuto("OL-SPL-SDEP-LClimb"));
        autoChooser.addOption("OR-SPL-RClimb", new PathPlannerAuto("OR-SPL-RClimb"));

        SmartDashboard.putData("autoChooser", autoChooser);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateControlsForAlliance(Alliance alliance) {
        stateController.updateAlliance(alliance);
    }
}
