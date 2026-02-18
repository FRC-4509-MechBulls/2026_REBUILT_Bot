package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{

    SparkMax leftMotor;
    SparkMax rightMotor;
    TalonFX intakeMotor;
    DutyCycleEncoder intakeExtensionEncoder;
    PIDController extensionController;
    double desiredPosition;
    double position;
    double currentSpeed;
    double calculatedSpeed;

    double debugSpeed = 0; // debug

    public IntakeSubsystem() {

        leftMotor = new SparkMax(Constants.IntakeConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.IntakeConstants.rightMotorID, MotorType.kBrushless);
        intakeMotor = new TalonFX(Constants.IntakeConstants.wheelMotorID);
        intakeExtensionEncoder = new DutyCycleEncoder(Constants.IntakeConstants.encoderChannel);
        extensionController = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
        desiredPosition = 0;
        position = 0;
        currentSpeed = 0;
        calculatedSpeed = 0;

        SmartDashboard.putNumber("DebugIntakeSpeed", debugSpeed);

    }

    @Override
    public void periodic(){

        setMotors(extensionController.calculate(intakeExtensionEncoder.get(), desiredPosition));

        debugSpeed = SmartDashboard.getNumber("DebugIntakeSpeed", 0);
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
    
}
