package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase{

    SparkMax rotationMotor;
    SparkMax extensionMotor;
    DutyCycleEncoder rotationEncoder;
    DutyCycleEncoder extensionEncoder;

    double desiredAngle;
    double desiredPosition;
    PIDController rotationController;
    PIDController extensionController;
    

    public ClimbSubsystem() {
        rotationMotor = new SparkMax(Constants.ClimbConstants.rotationMotorID, MotorType.kBrushless);
        extensionMotor = new SparkMax(Constants.ClimbConstants.extensionMotorID, MotorType.kBrushless);
        rotationEncoder = new DutyCycleEncoder(Constants.ClimbConstants.rotationEncoderChannel);
        extensionEncoder = new DutyCycleEncoder(Constants.ClimbConstants.extensionEncoderChannel);
        desiredAngle = 0;
        desiredPosition = 0;
        rotationController = new PIDController(Constants.ClimbConstants.rotationkP, Constants.ClimbConstants.rotationkI, Constants.ClimbConstants.rotationkD);
        extensionController = new PIDController(Constants.ClimbConstants.extensionkP, Constants.ClimbConstants.extensionkI, Constants.ClimbConstants.extensionkD);
    }

    public void setDesiredAngle(double angle){
        desiredAngle = angle;
    }
    public void setDesiredPosition(double position){
        desiredPosition = position;
    }

    public void periodic() {
        rotationMotor.set(rotationController.calculate(rotationEncoder.get(), desiredAngle));
        extensionMotor.set(extensionController.calculate(extensionEncoder.get(), desiredPosition));
    }

}
