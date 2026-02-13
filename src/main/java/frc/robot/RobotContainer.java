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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public final InstantCommand toggleIntake = new InstantCommand(()-> stateController.toggleIntake());   
    public final InstantCommand toggleHopper = new InstantCommand(()-> stateController.toggleHopper());
    public final InstantCommand toggleClimbRotate = new InstantCommand(()-> stateController.toggleClimbRotate());
    public final InstantCommand toggleClimbExtension = new InstantCommand(()-> stateController.toggleClimbExtension());   

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
        driverController.povUp().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-1) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.povDown().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(1) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.povLeft().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(-1) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.povRight().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(1) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // maybe remove this?
        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(()-> // While left trigger is held, drive while maintain a heading facing the target
            facingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                       .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                       .withTargetDirection(stateController.calculateAngleToAim())
        ));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        operatorController.b().onTrue(resetRobot);
        operatorController.leftTrigger().onTrue(doShooting);
        operatorController.leftTrigger().onFalse(stopShooting);
        operatorController.a().onTrue(toggleIntake);
        operatorController.x().onTrue(toggleHopper);
        driverController.rightBumper().onTrue(toggleClimbRotate);
        driverController.rightTrigger().onTrue(toggleClimbExtension);
        
        registerNamedCommands();
        createAutos();
    }
    public void registerNamedCommands() {
        NamedCommands.registerCommand("doNothing", new InstantCommand());
    }
    public void createAutos() {
        autoChooser.setDefaultOption("nothing", null);

        autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
        autoChooser.addOption("Auto2", new PathPlannerAuto("Auto2"));

        SmartDashboard.putData("autoChooser", autoChooser);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void updateControlsForAlliance(Alliance alliance) {
        stateController.updateAlliance(alliance);
    }
}
