package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class HubOffsetUtil {

//    private final Translation2d defaultBlueHubPose = FieldConstants.Hub.topCenterPoint.toTranslation2d();
//    private final Translation2d defaultRedHubPose = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

    private final Translation2d defaultBlueHubPose = new Translation2d(4.626,4.026);
    private final Translation2d defaultRedHubPose = new Translation2d(11.94, Units.inchesToMeters(158.85));

    private CommandSwerveDrivetrain drivetrain;

    public HubOffsetUtil(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public double calculateAirtime(double exitVelocityX, Alliance alliance, Translation2d defaultLocation) {

        double distance = Math.hypot(
            defaultLocation.getX() - drivetrain.getState().Pose.getX(),
            defaultLocation.getY() - drivetrain.getState().Pose.getY()
            );
        double airtime = distance/exitVelocityX;

        return airtime;
    }

    public Translation2d predictFutureLocation(double exitVelocityX, Alliance alliance) {

        Translation2d defaultLocation;
        if(alliance == Alliance.Blue){
            defaultLocation = defaultBlueHubPose;
        } else {
            defaultLocation = defaultRedHubPose;
        }

        ChassisSpeeds driveSpeed = drivetrain.getState().Speeds;
        double airtime = calculateAirtime(exitVelocityX, alliance, defaultLocation);

        Translation2d distanceMoved = new Translation2d(
            (-driveSpeed.vxMetersPerSecond * airtime),
            (-driveSpeed.vyMetersPerSecond * airtime)
        );
        if(alliance == Alliance.Red){
            distanceMoved = new Translation2d(
                (driveSpeed.vxMetersPerSecond * airtime),
                (driveSpeed.vyMetersPerSecond * airtime)
            );
        }

        Translation2d futureLocation = (defaultLocation.plus(distanceMoved));

        return futureLocation;
    }
}
