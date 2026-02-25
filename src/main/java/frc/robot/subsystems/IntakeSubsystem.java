package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    DutyCycleEncoder intakeExtensionEncoder;
    PIDController extensionController;
    int rotationCount;
    double desiredPosition;
    double currentPosition;
    double lastPosition;
    double currentSpeed;

    // Testing
    double debugSpeed = 0; 
    double extensionkP = 0;
    double extensionkI = 0;
    double extensionkD = 0;

    public IntakeSubsystem() {

        // Motor Intialization
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
        intakeExtensionEncoder = new DutyCycleEncoder(Constants.IntakeConstants.encoderChannel);
        extensionController = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
        rotationCount = 0;
        desiredPosition = 0;
        currentPosition = 0;
        lastPosition = intakeExtensionEncoder.get();
        currentSpeed = 0;

        // Testing
        SmartDashboard.putNumber("DebugIntakeSpeed", debugSpeed);
        SmartDashboard.putNumber("HopperExtensionkP", extensionkP);
        SmartDashboard.putNumber("HopperExtensionkI", extensionkI);
        SmartDashboard.putNumber("HopperExtensionkD", extensionkD);
        SmartDashboard.putNumber("DesiredHopperExtension", desiredPosition);

    }

    @Override
    public void periodic(){

        currentPosition = getContinuousPosition();
        setMotors(extensionController.calculate(currentPosition, desiredPosition));

        // Testing
        debugSpeed = SmartDashboard.getNumber("DebugIntakeSpeed", 0);
        desiredPosition = SmartDashboard.getNumber("DesiredHopperExtension", 0);
        extensionController.setPID(
                            SmartDashboard.getNumber("HopperExtensionkP", 0),
                            SmartDashboard.getNumber("HopperExtensionkI", 0), 
                            SmartDashboard.getNumber("HopperExtensionkD", 0)
                            );
    }

    public void setMotors(double speed){
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void intake(boolean intake){
        if(intake){
            //intakeMotor.set(Constants.IntakeConstants.intakeWheelSpeed);
            intakeMotor.set(debugSpeed);
        } else {
            intakeMotor.set(0);
        }
    }

    public void setPosition(double newPosition){
        desiredPosition = newPosition;
    }

    public double getContinuousPosition() {
        double currentPosition = intakeExtensionEncoder.get();

        if(lastPosition > 0.85 && currentPosition < 0.1) {
            rotationCount++;
        } else if (lastPosition < 0.1 && currentPosition > 0.85) {
            rotationCount--;
        }

        lastPosition = currentPosition;

        return rotationCount + currentPosition;
    }
    
}
