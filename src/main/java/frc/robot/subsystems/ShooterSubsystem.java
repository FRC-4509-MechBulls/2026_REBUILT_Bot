package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
    // Motors
    SparkMax hoodMotor;
    SparkMax flywheelMotor;
    SparkMax indexerMotor;
    SparkMaxConfig sparkMaxConfig;

    // Feedback Control
    PIDController speedController;
    double desiredSpeed;
    double currentSpeed;
    double calculatedSpeed;
    DutyCycleEncoder hoodEncoder;
    PIDController hoodController;
    double currentAngle;
    double desiredAngle;

    // Testing
    double indexerSpeed = 0;
    double flywheelkP = 0;;
    double flywheelkI = 0;
    double flywheelkD = 0;
    double hoodkP = 0;
    double hoodkI = 0;
    double hoodkD = 0;

    public ShooterSubsystem() {

        // Motor Initialization
        hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorID, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Constants.ShooterConstants.flywheelMotorID, MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.ShooterConstants.indexerMotorID, MotorType.kBrushless);
        
        // Motor Config
        sparkMaxConfig = new SparkMaxConfig();
            sparkMaxConfig.idleMode(IdleMode.kBrake);
            sparkMaxConfig.smartCurrentLimit(40);
            sparkMaxConfig.secondaryCurrentLimit(50);
            sparkMaxConfig.voltageCompensation(12);
        hoodMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Feedback Control Initialization        
        speedController = new PIDController(Constants.ShooterConstants.speedkP, Constants.ShooterConstants.speedkI, Constants.ShooterConstants.speedkD);
        desiredSpeed = 0;
        currentSpeed = 0;
        calculatedSpeed = 0;
        hoodEncoder = new DutyCycleEncoder(Constants.ShooterConstants.hoodEncoderChannel);
        hoodController = new PIDController(Constants.ShooterConstants.hoodkP, Constants.ShooterConstants.hoodkI, Constants.ShooterConstants.hoodkD);
        currentAngle = 0;
        desiredAngle = 0;

        // Testing
        indexerSpeed = 0;
        SmartDashboard.putNumber("desiredShooterAngle", desiredAngle);
        SmartDashboard.putNumber("desiredFlywheelSpeed", desiredSpeed);
        SmartDashboard.putNumber("indexerSpeed", indexerSpeed);
        SmartDashboard.putNumber("flywheelkP", flywheelkP);
        SmartDashboard.putNumber("flywheelkI", flywheelkI);
        SmartDashboard.putNumber("flywheelkD", flywheelkD);
        SmartDashboard.putNumber("hoodkP", hoodkP);
        SmartDashboard.putNumber("hoodkI", hoodkI);
        SmartDashboard.putNumber("hoodkD", hoodkD);

    }

    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed;
    }
    public void setShooterSpeed(double speed) {
        flywheelMotor.set(speed);
    }
    public void setHoodAngle(double angle) {
        desiredAngle = angle;
    }
    public void setHoodSpeed(double speed) {
        hoodMotor.set(speed);
    }
    public void setIndexer(boolean load) {
        if(load){
//            indexerMotor.set(Constants.ShooterConstants.indexerLoadSpeed);
            indexerMotor.set(indexerSpeed);
        } else {
            indexerMotor.set(0);
        }
    }

    public void periodic(){
        setShooterSpeed(currentSpeed + speedController.calculate(currentSpeed, desiredSpeed)); // might go wrong direction idk
        setHoodSpeed(hoodController.calculate(hoodEncoder.get(), desiredAngle));

        // Testing
        desiredAngle = SmartDashboard.getNumber("desiredShooterAngle", 0);
        desiredSpeed = SmartDashboard.getNumber("desiredFlywheelSpeed", 0);
        indexerSpeed = SmartDashboard.getNumber("indexerSpeed", 0);
        speedController.setPID(
            SmartDashboard.getNumber("flywheelkP", 0), 
            SmartDashboard.getNumber("flywheelkI", 0), 
            SmartDashboard.getNumber("flywheelkD", 0));
        hoodController.setPID(
            SmartDashboard.getNumber("hoodkP", 0), 
            SmartDashboard.getNumber("hoodkI", 0), 
            SmartDashboard.getNumber("hoodkD", 0));
    }

    

}
