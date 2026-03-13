package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase{

    // Motors
    SparkMax rotationMotor;
    SparkMax extensionMotor;
    SparkMaxConfig sparkMaxConfig;

    // Feedback Control
    DutyCycleEncoder rotationEncoder;
    DutyCycleEncoder extensionEncoder;
    double desiredAngle;
    double desiredPosition;
    PIDController rotationController;
    PIDController extensionController;
    
    // Testing
    double rotationkP = 0;
    double rotationkI = 0;
    double rotationkD = 0;
    double extensionkP = 0;
    double extensionkI = 0;
    double extensionkD = 0;

    public ClimbSubsystem() {
/** 
        // Motor Initialization
        rotationMotor = new SparkMax(Constants.ClimbConstants.rotationMotorID, MotorType.kBrushless);
        extensionMotor = new SparkMax(Constants.ClimbConstants.extensionMotorID, MotorType.kBrushless);
        
        // Motor Config
        sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig.idleMode(IdleMode.kBrake);
            sparkMaxConfig.smartCurrentLimit(40);
            sparkMaxConfig.secondaryCurrentLimit(50);
            sparkMaxConfig.voltageCompensation(12);
        rotationMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extensionMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Feedback Control Initialization
        rotationEncoder = new DutyCycleEncoder(Constants.ClimbConstants.rotationEncoderChannel);
        extensionEncoder = new DutyCycleEncoder(Constants.ClimbConstants.extensionEncoderChannel);
        desiredAngle = 0;
        desiredPosition = 0;
        rotationController = new PIDController(Constants.ClimbConstants.rotationkP, Constants.ClimbConstants.rotationkI, Constants.ClimbConstants.rotationkD);
        extensionController = new PIDController(Constants.ClimbConstants.extensionkP, Constants.ClimbConstants.extensionkI, Constants.ClimbConstants.extensionkD);

        // Testing
        SmartDashboard.putNumber("ClimbRotationkP", rotationkP);
        SmartDashboard.putNumber("ClimbRotationkI", rotationkI);
        SmartDashboard.putNumber("ClimbRotationkD", rotationkD);
        SmartDashboard.putNumber("ClimbExtensionkP", extensionkP);
        SmartDashboard.putNumber("ClimbExtensionkI", extensionkI);
        SmartDashboard.putNumber("ClimbExtensionkD", extensionkD);
        SmartDashboard.putNumber("desiredClimbAngle", desiredAngle);
        SmartDashboard.putNumber("desiredClimbExtension", desiredPosition);
*/
    }

    public void setDesiredAngle(double angle){
//        desiredAngle = angle;
    }
    public void setDesiredPosition(double position){
//        desiredPosition = position;
    }

    public void periodic() {
/** 
        rotationMotor.set(rotationController.calculate(rotationEncoder.get(), desiredAngle));
        extensionMotor.set(extensionController.calculate(extensionEncoder.get(), desiredPosition));

        // Testing
        rotationController.setPID(SmartDashboard.getNumber("ClimbRotationkP", 0), SmartDashboard.getNumber("ClimbRotationkI", 0), SmartDashboard.getNumber("ClimbRotationkD", 0));
        extensionController.setPID(SmartDashboard.getNumber("ClimbExtensionkP", 0), SmartDashboard.getNumber("ClimbExtensionkI", 0), SmartDashboard.getNumber("ClimbExtensionkD", 0));
        desiredAngle = SmartDashboard.getNumber("desiredClimbAngle", 0);
        desiredPosition = SmartDashboard.getNumber("desiredClimbExtension", 0);
*/    
    }

}
