package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetOffsetUtil {
    
    private final Translation2d defaultBlueAlliance1 = Constants.PoseConstants.blueAlliance1.getTranslation();
    private final Translation2d defaultBlueAlliance2 = Constants.PoseConstants.blueAlliance2.getTranslation();
    private final Translation2d defaultRedAlliance1 = Constants.PoseConstants.redAlliance1.getTranslation();
    private final Translation2d defaultRedAlliance2 = Constants.PoseConstants.redAlliance2.getTranslation();

    private final double dampeningFactorX = Constants.DriveConstants.aimDampeningFactorX;
    private final double dampeningFactorY = Constants.DriveConstants.aimDampeningFactorY;

    private CommandSwerveDrivetrain drivetrain;

    public TargetOffsetUtil(CommandSwerveDrivetrain drivetrain) {
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

        double y = drivetrain.getState().Pose.getY();

        Translation2d defaultLocation;
        
        if(alliance == Alliance.Blue){
            if(y < 4.035) {
                defaultLocation = defaultBlueAlliance1;
            } else{
                defaultLocation = defaultBlueAlliance2;
            }
        } else {
            if(y < 4.035) {
                defaultLocation = defaultRedAlliance1;
            } else{
                defaultLocation = defaultRedAlliance2;
            }
        }

        ChassisSpeeds driveSpeed = drivetrain.getState().Speeds;
        double airtime = calculateAirtime(exitVelocityX, alliance, defaultLocation);

        Translation2d distanceMoved = new Translation2d(
            (driveSpeed.vxMetersPerSecond * airtime * dampeningFactorX),
            (driveSpeed.vyMetersPerSecond * airtime * dampeningFactorY)
        );
        if(alliance == Alliance.Red){
            distanceMoved = new Translation2d(
                (-driveSpeed.vxMetersPerSecond * airtime * dampeningFactorX),
                (-driveSpeed.vyMetersPerSecond * airtime * dampeningFactorY)
            );
        }

        Translation2d futureLocation = (defaultLocation.plus(distanceMoved));

        return futureLocation;
    }
}
