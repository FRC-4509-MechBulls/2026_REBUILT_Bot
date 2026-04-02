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
import edu.wpi.first.wpilibj.XboxController;
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

    double intakeWheelSpeed;

    XboxController controller = new XboxController(1);

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
        intakeExtensionEncoder = new DutyCycleEncoder(Constants.IntakeConstants.encoderChannel);
        extensionController = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
        rotationCount = 0;
        desiredPosition = 0;
        currentPosition = 0;
        lastPosition = intakeExtensionEncoder.get();
        currentSpeed = 0;

        intakeWheelSpeed = Constants.IntakeConstants.intakeWheelSpeed;
        SmartDashboard.putNumber("DesiredHopperExtension", desiredPosition);

    }

    @Override
    public void periodic(){

  //      setMotors(extensionController.calculate(getContinuousPosition(), desiredPosition));

        SmartDashboard.putNumber("IntakePosition", getContinuousPosition());
        
        // Debug
        
        

        
    }

    public void controlIntake(double leftY, double rightY){
        intakeMotor.set(rightY);
        setMotors(leftY*6);
    }

    public void setMotors(double speed){
        leftMotor.setVoltage(-speed);
        rightMotor.setVoltage(speed);
    }

    public void intake(boolean intake){
        if(intake){
            intakeMotor.set(intakeWheelSpeed);
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
