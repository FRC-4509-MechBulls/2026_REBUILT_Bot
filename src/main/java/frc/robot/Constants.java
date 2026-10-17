package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    //hi
    public class IntakeConstants {
        public static final int leftMotorID = 19;
        public static final int rightMotorID = 30;
        public static final int wheelMotorID = 21;
        public static final double intakeWheelSpeed = 1;

        public static final boolean hopperLimitsEnabled = false;
        public static final double hopperExtendedRotations = 0;
        public static final double hopperRetractedRotations = 0;
        public static final double hopperPositionTolerance = 0.5;
    }

    public class ShooterConstants {
        public static final int flywheelMotorID = 20;
        public static final int indexerMotorID = 25;

        // ---------------- Flywheel (NEO on SparkMax) ----------------
        // Runs open loop, so all speeds are motor output from 0 to 1.
        public static final boolean flywheelInverted = false;   // flip if positive output spins the wheel backwards
        public static final boolean flywheelBrakeMode = false;  // coast is easier on the motor; brake stops the wheel hard
        public static final int flywheelCurrentLimit = 50;      // amps (smart current limit)
        public static final int flywheelSecondaryCurrentLimit = 60; // amps (hard limit)
        public static final double flywheelReverseSpeed = 0.5;  // output used when spinning the shooter backwards
        public static final double flywheelMaxRPM = 3000;
        public static final double flywheelReadyFraction = 0.85;
        public static final double flywheelReadyDebounce = 0.1;
        public static final double spinUpTimeout = 1.0;
        // --------------------------------------------------------------------

        public static final boolean flywheelClosedLoop = false;
        public static final double flywheelkP = 0.0002;
        public static final double flywheelkV = 12.0 / flywheelMaxRPM;

        public static final double indexerLoadSpeed = -.8;
        public static final double indexerJamCurrent = 35;
        public static final double indexerJamVelocity = 5;
        public static final double indexerJamTime = 0.25;
        public static final double indexerUnjamTime = 0.3;

        public static final double simpleShootingSpeed = .75;               // operator right trigger
        public static final double simpleShootingSpeedHopperExtended = .75; // operator right trigger with hopper out

        public static final double scoringHoodAngle = 30;

        public static final double[] shotDistances = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 3.93};
        public static final double[] shotOutputs = {0.506, 0.62, 0.715, 0.8, 0.877, 0.946, 1.0};
    }

    public class DriveConstants {
        public static final double autonTranslationkP = 5;
        public static final double autonTranslationkI = 0;
        public static final double autonTranslationkD = 0;
        public static final double autonRotationkP = 5;
        public static final double autonRotationkI = 0;
        public static final double autonRotationkD = 0;
        public static final double aimDampeningFactorX = 0.5;
        public static final double aimDampeningFactorY = 0.5;
        public static final double aimkP = 5;
        public static final double aimkI = 0;
        public static final double aimkD = 0;

        public static final double bumpTravelTime = 1.3;

        public static final double driveAccelLimit = 8.0;
    }

    public class PoseConstants {
        public static final Pose2d blueHub = new Pose2d(new Translation2d(4.626,4.034), new Rotation2d());
        public static final Pose2d redHub =  new Pose2d(new Translation2d(FieldConstants.fieldLength - 4.626, 4.034), new Rotation2d());

        public static final Pose2d blueAlliance1 =  new Pose2d(new Translation2d(2.31, 2.017), new Rotation2d());
        public static final Pose2d blueAlliance2 =  new Pose2d(new Translation2d(2.31, 6.05), new Rotation2d());

        public static final Pose2d redAlliance1 =  new Pose2d(new Translation2d(14.23, 2.017), new Rotation2d());
        public static final Pose2d redAlliance2 =  new Pose2d(new Translation2d(14.23, 6.05), new Rotation2d());

        public static final Pose2d knownBlueHubRobotPose = new Pose2d(new Translation2d(3.601, 4.034), new Rotation2d());
        public static final Pose2d knownRedHubRobotPose = new Pose2d(new Translation2d(FieldConstants.fieldLength - 3.601, 4.034), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d knownBlueLeftTrenchRobotPose = new Pose2d(new Translation2d(3.662, 6.663), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d knownRedLeftTrenchRobotPose = new Pose2d(new Translation2d(FieldConstants.fieldLength - 3.662, FieldConstants.fieldWidth - 6.663), new Rotation2d());
        public static final Pose2d knownBlueRightTrenchRobotPose = new Pose2d(new Translation2d(3.662, FieldConstants.fieldWidth - 6.663), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d knownRedRightTrenchRobotPose = new Pose2d(new Translation2d(FieldConstants.fieldLength - 3.662, 6.663), new Rotation2d());
        public static final Pose2d leftBumpStartBlue = new Pose2d(new Translation2d(3.525, 5.568), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d rightBumpStartBlue = new Pose2d(new Translation2d(3.525, 2.47), new Rotation2d(Units.degreesToRadians(180)));
//        public static final Pose2d neutralZone1 =  new Pose2d(new Translation2d(), new Rotation2d());
//        public static final Pose2d neutralZone2 =  new Pose2d(new Translation2d(), new Rotation2d());


    }

    public static boolean disableHAL = false;
    public static void disableHAL() {
        disableHAL = true;
    }



}
