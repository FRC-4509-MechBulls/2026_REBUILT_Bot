package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    SparkMax hoodMotor;
    SparkMax flywheelMotor;
    PIDController speedController;
    double desiredSpeed;
    double currentSpeed;
    double calculatedSpeed;

    public ShooterSubsystem() {

        hoodMotor = new SparkMax(0, MotorType.kBrushless);
        flywheelMotor = new SparkMax(0, MotorType.kBrushless);
        speedController = new PIDController(0, 0, 0);
        desiredSpeed = 0;
        currentSpeed = 0;
        calculatedSpeed = 0;

    }

    public void setDesiredSpeed(double speed){
        desiredSpeed = speed;
    }

    public void periodic(){
        calculatedSpeed = speedController.calculate(currentSpeed, desiredSpeed);

        setShooterSpeed(calculatedSpeed);
    }

    public void setShooterSpeed(double speed){
        hoodMotor.set(speed);
        flywheelMotor.set(speed);
    }

}
