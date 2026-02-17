package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HubOffsetUtil {

    private final Translation2d defaultBlueHubPose = Constants.PoseConstants.blueHub.getTranslation();
    private final Translation2d defaultRedHubPose = Constants.PoseConstants.redHub.getTranslation();

    private final double dampeningFactorX = Constants.DriveConstants.aimDampeningFactorX;
    private final double dampeningFactorY = Constants.DriveConstants.aimDampeningFactorY;

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
            (-driveSpeed.vxMetersPerSecond * airtime * dampeningFactorX),
            (-driveSpeed.vyMetersPerSecond * airtime * dampeningFactorY)
        );
        if(alliance == Alliance.Red){
            distanceMoved = new Translation2d(
                (driveSpeed.vxMetersPerSecond * airtime * dampeningFactorX),
                (driveSpeed.vyMetersPerSecond * airtime * dampeningFactorY)
            );
        }

        Translation2d futureLocation = (defaultLocation.plus(distanceMoved));

        return futureLocation;
    }
}
