package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.AllianceFlipUtil;

public class Constants {
    //hi
    public class IntakeConstants {
        public static final int leftMotorID = 17;
        public static final int rightMotorID = 19;
        public static final int wheelMotorID = 21;
        public static final int encoderChannel = 7;
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0.5;
        public static final double extendedPosition = 2.8;
        public static final double retractedPosition = 1.1;
        public static final double intakeWheelSpeed = 1;
    }

    public class ShooterConstants {
        public static final int hoodMotorID = 30;
        public static final int flywheelMotorID = 20;
        public static final int indexerMotorID = 25;
        public static final int hoodEncoderChannel = 6;
        public static final double speedkP = 5;
        public static final double speedkI = 0;
        public static final double speedkD = 0.5;
        public static final double speedkS = 0.0;
        public static final double speedkV = 0.04; // make sure this is tuned to RPM
        public static final double speedkA = 22.83; // 22.83
        public static final double hoodkP = 5;
        public static final double hoodkI = 0;
        public static final double hoodkD = 0;

        public static final long windUpTime = 2500; //ms
        public static final long recoveryTime = 500;
        public static final double indexerLoadSpeed = -.7;

        public static final double verticalDisplacement = 1.3; 
        public static final double simpleShootingSpeed = 0.5;
        public static final double simpleShootingSpeedHopperExtended = 0.5;
        public static final double simpleShootingAngle = 70;

        public static final double maxDistance = 7;
        public static final double minDistance = 1;
        public static final double maxHoodAngle = 70;
        public static final double minHoodAngle = 70;

        public static final double scoringHoodAngle = 70;
        public static final double passingHoodAngle = 50;

        public static final double maxProjectileHeight = 5;

        public static final double maxFlywheelRPM = 5906;
        public static final double shotRPMDampener = 1.4;

        public static final double[] correspondingExitVelocity = {1.16, 2.33, 3.49, 4.65, 5.82, 6.98, 8.15, 9.31, 10.47, 11.67};
        public static final double[] correspondingMotorOutput = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
        
    }

    public class ClimbConstants {
        public static final int rotationMotorID = 50;
        public static final int extensionMotorID = 49;
        public static final int rotationEncoderChannel = 4;
        public static final int extensionEncoderChannel = 5;
        public static final double rotationkP = 0;
        public static final double rotationkI = 0;
        public static final double rotationkD = 0;
        public static final double extensionkP = 0;
        public static final double extensionkI = 0;
        public static final double extensionkD = 0;
        public static final double climbRestingAngle = 0;
        public static final double climbReadyAngle = 0;
        public static final double climbRetractedDistance = 0;
        public static final double climbExtendedDistance = 0;
    }

    public class VisionConstants {
        public static Matrix<N3, N1> visionStd = VecBuilder.fill(.5,.5,Units.degreesToRadians(5));
        public static final Transform3d robotToFrontRightCamera = new Transform3d(
                                                        Units.inchesToMeters(11.6035),
                                                        Units.inchesToMeters(-11.6835),
                                                        Units.inchesToMeters(8.124175),
                                                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45))
                                                        );
        public static final Transform3d robotToBackLeftCamera = new Transform3d(
                                                        Units.inchesToMeters(-11.6035),
                                                        Units.inchesToMeters(11.6835),
                                                        Units.inchesToMeters(8.124175),
                                                        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(135))
                                                        );
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

        public static final double bumpTravelDifference = 1.738; // inches
        public static final double distanceTraveledOverBump = 1.287; // meters
        public static final double bumpTravelTime = 1.3;

        public static final Rotation2d blueForwardX = new Rotation2d(0);
        public static final Rotation2d redForwardX = new Rotation2d(Units.degreesToRadians(180));
    
        public static final Pose2d OL_BUMP_NZ_BLUE = new Pose2d(5.591, 5.525, new Rotation2d(Math.PI));
        public static final Pose2d OR_BUMP_NZ_BLUE = new Pose2d(5.591, 2.47, new Rotation2d(Math.PI));
        public static final Pose2d OL_BUMP_AZ_BLUE = new Pose2d(3.525, 5.568, new Rotation2d(0));
        public static final Pose2d OR_BUMP_AZ_BLUE = new Pose2d(3.525, 2.47, new Rotation2d(0));
        public static final Pose2d OL_BUMP_NZ_RED = AllianceFlipUtil.apply(OL_BUMP_NZ_BLUE);
        public static final Pose2d OR_BUMP_NZ_RED = AllianceFlipUtil.apply(OR_BUMP_NZ_BLUE);
        public static final Pose2d OL_BUMP_AZ_RED = AllianceFlipUtil.apply(OL_BUMP_AZ_BLUE);
        public static final Pose2d OR_BUMP_AZ_RED = AllianceFlipUtil.apply(OR_BUMP_AZ_BLUE);
    }

    public class PoseConstants {
        public static final Pose2d blueHub = new Pose2d(new Translation2d(4.626,4.034), new Rotation2d());
        public static final Pose2d redHub =  new Pose2d(new Translation2d(11.94, 4.034), new Rotation2d());

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
//        public static final Pose2d neutralZone1 =  new Pose2d(new Translation2d(), new Rotation2d());
//        public static final Pose2d neutralZone2 =  new Pose2d(new Translation2d(), new Rotation2d());

        
    }

    public static boolean disableHAL = false;
    public static void disableHAL() {
        disableHAL = true;
    }



}
