package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{

    // Motors
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMaxConfig sparkMaxConfig;
    TalonFX intakeMotor;
    TalonFXConfiguration talonFXConfiguration;

    // Feedback Control
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    double requestedHopperVolts;

    double intakeWheelSpeed;
    double intakeWheelOutput;

    public IntakeSubsystem() {

        // Motor Initialization
        leftMotor = new SparkMax(Constants.IntakeConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.IntakeConstants.rightMotorID, MotorType.kBrushless);
        intakeMotor = new TalonFX(Constants.IntakeConstants.wheelMotorID);

        // Motor Config
        sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig.idleMode(IdleMode.kBrake);
            sparkMaxConfig.smartCurrentLimit(40);
            sparkMaxConfig.secondaryCurrentLimit(50);
            sparkMaxConfig.voltageCompensation(12);
        leftMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        talonFXConfiguration = new TalonFXConfiguration()
                                    .withCurrentLimits(new CurrentLimitsConfigs()
                                                        .withStatorCurrentLimit(40)
                                                        .withSupplyCurrentLimit(40)
                                    );
        intakeMotor.getConfigurator().apply(talonFXConfiguration);

        // Feedback Control Initialization
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        requestedHopperVolts = 0;

        intakeWheelSpeed = Constants.IntakeConstants.intakeWheelSpeed;
        intakeWheelOutput = 0;

    }

    @Override
    public void periodic(){

        applyHopperVoltage();

        SmartDashboard.putNumber("HopperPosition", getHopperPosition());

    }

    public void controlIntake(double leftY, double rightY){
        setIntakeWheels(rightY);
        setMotors(leftY*6);
    }

    public void setMotors(double speed){
        requestedHopperVolts = speed;
        applyHopperVoltage();
    }

    private void applyHopperVoltage() {
        double volts = requestedHopperVolts;

        if(Constants.IntakeConstants.hopperLimitsEnabled) {
            boolean pastExtended = volts < 0 && getHopperPosition() >= Constants.IntakeConstants.hopperExtendedRotations;
            boolean pastRetracted = volts > 0 && getHopperPosition() <= Constants.IntakeConstants.hopperRetractedRotations;
            if(pastExtended || pastRetracted) {
                volts = 0;
            }
        }

        leftMotor.setVoltage(-volts);
        rightMotor.setVoltage(volts);
    }

    public double getHopperPosition() {
        return (leftEncoder.getPosition() - rightEncoder.getPosition()) / 2;
    }

    public boolean isHopperExtended() {
        return Constants.IntakeConstants.hopperLimitsEnabled
            && getHopperPosition() >= Constants.IntakeConstants.hopperExtendedRotations - Constants.IntakeConstants.hopperPositionTolerance;
    }

    public boolean isHopperRetracted() {
        return Constants.IntakeConstants.hopperLimitsEnabled
            && getHopperPosition() <= Constants.IntakeConstants.hopperRetractedRotations + Constants.IntakeConstants.hopperPositionTolerance;
    }

    public void zeroHopper() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void intake(boolean intake){
        if(intake){
            setIntakeWheels(intakeWheelSpeed);
        } else {
            setIntakeWheels(0);
        }
    }

    private void setIntakeWheels(double output) {
        intakeWheelOutput = output;
        intakeMotor.set(output);
    }

    public boolean isIntaking() {
        return Math.abs(intakeWheelOutput) > 0.05;
    }

}
