package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase{

    // Motors
    SparkMax flywheelMotor;
    TalonFX indexerMotor;
    SparkMaxConfig flywheelConfig;
    TalonFXConfiguration indexerConfig;

    // Flywheel runs open loop, output is -1 to 1
    double desiredSpeed = 0;
    RelativeEncoder flywheelEncoder;
    Debouncer atSpeedDebouncer = new Debouncer(Constants.ShooterConstants.flywheelReadyDebounce, DebounceType.kRising);
    boolean atSpeed = false;

    double hoodAngle = Constants.ShooterConstants.scoringHoodAngle;
    double indexerSpeed = 0;

    public ShooterSubsystem() {

        // Motor Initialization
        flywheelMotor = new SparkMax(Constants.ShooterConstants.flywheelMotorID, MotorType.kBrushless);
        indexerMotor = new TalonFX(Constants.ShooterConstants.indexerMotorID, TunerConstants.kCANBus); // indexer is on the DriveTrain CANivore

        // Motor Config
        flywheelConfig = new SparkMaxConfig();
            flywheelConfig.inverted(Constants.ShooterConstants.flywheelInverted);
            flywheelConfig.idleMode(Constants.ShooterConstants.flywheelBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            flywheelConfig.smartCurrentLimit(Constants.ShooterConstants.flywheelCurrentLimit);
            flywheelConfig.secondaryCurrentLimit(Constants.ShooterConstants.flywheelSecondaryCurrentLimit);
            flywheelConfig.voltageCompensation(12); // keeps open loop shots consistent as the battery drains
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelEncoder = flywheelMotor.getEncoder();
        indexerConfig = new TalonFXConfiguration()
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true));
        indexerMotor.getConfigurator().apply(indexerConfig);

        indexerSpeed = Constants.ShooterConstants.indexerLoadSpeed;

        // Testing
        SmartDashboard.putNumber("indexerSpeed", indexerSpeed);

    }

    /** Open loop flywheel output from -1 to 1. Positive shoots, negative spins the shooter backwards. */
    public void setDesiredSpeed(double speed) {
        desiredSpeed = MathUtil.clamp(speed, -1, 1);
        if(desiredSpeed == 0) {
            flywheelMotor.stopMotor();
        } else {
            flywheelMotor.set(desiredSpeed);
        }
    }
    /** Spin the shooter the opposite direction (unjam). */
    public void reverse() {
        setDesiredSpeed(-Constants.ShooterConstants.flywheelReverseSpeed);
    }
    public void stop() {
        setDesiredSpeed(0);
    }
    public double getDesiredSpeed() {
        return desiredSpeed;
    }
    public double getFlywheelRPM() {
        return flywheelEncoder.getVelocity();
    }
    public boolean isAtSpeed() {
        return atSpeed;
    }

    public void setIndexer(boolean load) {
        if(load){
            indexerMotor.set(indexerSpeed);
        } else {
            indexerMotor.set(0);
        }
    }
    public void reverseIndexer(){
        indexerMotor.set(-indexerSpeed);
    }

    @Override
    public void periodic(){
        double expectedRPM = desiredSpeed * Constants.ShooterConstants.flywheelMaxRPM;
        boolean upToSpeed = desiredSpeed > 0
                && getFlywheelRPM() >= expectedRPM * Constants.ShooterConstants.flywheelReadyFraction;
        atSpeed = atSpeedDebouncer.calculate(upToSpeed);

        SmartDashboard.putNumber("FlywheelOutput", desiredSpeed);
        SmartDashboard.putNumber("FlywheelRPM", getFlywheelRPM());
        SmartDashboard.putBoolean("ShooterReady", atSpeed);
    }

    public double getCurrentHoodAngle() {
        return hoodAngle;
    }
}
