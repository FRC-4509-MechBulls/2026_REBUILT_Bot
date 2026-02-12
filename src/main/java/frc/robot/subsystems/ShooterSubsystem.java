package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
    SparkMax hoodMotor;
    SparkMax flywheelMotor;
    SparkMax indexerMotor;
    PIDController speedController;
    double desiredSpeed;
    double currentSpeed;
    double calculatedSpeed;
    DutyCycleEncoder hoodEncoder;
    PIDController hoodController;
    double currentAngle;
    double desiredAngle;



    public ShooterSubsystem() {

        hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorID, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Constants.ShooterConstants.flywheelMotorID, MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.ShooterConstants.indexerMotorID, MotorType.kBrushless);
        speedController = new PIDController(Constants.ShooterConstants.speedkP, Constants.ShooterConstants.speedkI, Constants.ShooterConstants.speedkD);
        desiredSpeed = 0;
        currentSpeed = 0;
        calculatedSpeed = 0;
        hoodEncoder = new DutyCycleEncoder(Constants.ShooterConstants.hoodEncoderChannel);
        hoodController = new PIDController(Constants.ShooterConstants.hoodkP, Constants.ShooterConstants.hoodkI, Constants.ShooterConstants.hoodkD);
        currentAngle = 0;
        desiredAngle = 0;

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
            indexerMotor.set(Constants.ShooterConstants.indexerLoadSpeed);
        } else {
            indexerMotor.set(0);
        }
    }

    public void periodic(){
        setShooterSpeed(speedController.calculate(currentSpeed, desiredSpeed));
        setHoodSpeed(hoodController.calculate(hoodEncoder.get(), desiredAngle));
    }

    

}
