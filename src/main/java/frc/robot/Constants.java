package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public class IntakeConstants {
        public static final int leftMotorID = 0;
        public static final int rightMotorID = 1;
        public static final int wheelMotorID = 7;
        public static final int encoderChannel = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double extendedPosition = 0;
        public static final double retractedPosition = 0;
        public static final double intakeWheelSpeed = 0;
    }

    public class ShooterConstants {
        public static final int hoodMotorID = 2;
        public static final int flywheelMotorID = 3;
        public static final int indexerMotorID = 6;
        public static final int hoodEncoderChannel = 8;
        public static final double speedkP = 0;
        public static final double speedkI = 0;
        public static final double speedkD = 0;
        public static final double hoodkP = 0;
        public static final double hoodkI = 0;
        public static final double hoodkD = 0;
        public static final double hoodAngle1 = 45;
        public static final double hoodAngle2 = 60;
        public static final long windUpTime = 1000; //ms
        public static final double indexerLoadSpeed = 1;

        public static final double maxFlyWheelSpeed = 10;
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
    }

    public class DriveConstants {
        public static Matrix<N3, N1> visionStd = VecBuilder.fill(.05,.05,Units.degreesToRadians(5));
        public static final double aimDampeningFactorX = 0.75;
        public static final double aimDampeningFactorY = 0.75;
        public static final double aimkP = 5;
        public static final double aimkI = 0;
        public static final double aimkD = 0;
    }

    public static boolean disableHAL = false;
    public static void disableHAL() {
        disableHAL = true;
    }



}
