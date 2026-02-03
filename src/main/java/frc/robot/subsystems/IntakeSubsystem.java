package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{

    SparkMax leftMotor;
    SparkMax rightMotor;
    DutyCycleEncoder intakeExtensionEncoder;
    PIDController extensionController;
    double desiredPosition;
    double position;
    double currentSpeed;
    double calculatedSpeed;

    public IntakeSubsystem() {

        leftMotor = new SparkMax(Constants.IntakeConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.IntakeConstants.rightMotorID, MotorType.kBrushless);
        intakeExtensionEncoder = new DutyCycleEncoder(Constants.IntakeConstants.encoderChannel);
        extensionController = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
        desiredPosition = 0;
        position = 0;
        currentSpeed = 0;
        calculatedSpeed = 0;
        
    }

    @Override
    public void periodic(){

        calculatedSpeed = extensionController.calculate(intakeExtensionEncoder.get(), desiredPosition);

        setMotors(calculatedSpeed);

    }

    public void setMotors(double speed){
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void setPosition(double newPosition){
        desiredPosition = newPosition;
    }
    
}
