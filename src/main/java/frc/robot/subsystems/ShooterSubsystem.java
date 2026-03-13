package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
    // Motors
    SparkMax hoodMotor;
    SparkMax flywheelMotor;
    TalonFX indexerMotor;
    SparkMaxConfig flywheelConfig;
    SparkMaxConfig hoodConfig;

    // Flywheel Control
    RelativeEncoder flywheelEncoder;
    SparkClosedLoopController flywheelController;
    SimpleMotorFeedforward speedFeedForward;

    double desiredSpeed;
    double maxRPM = Constants.ShooterConstants.maxFlywheelRPM;


    DutyCycleEncoder hoodEncoder;
    PIDController hoodController;
    double currentAngle;
    double desiredAngle;

    double indexerSpeed = 0;

    // Testing
    double hoodkP = 5;
    double hoodkI = 0;
    double hoodkD = 0;

    XboxController controller = new XboxController(2);

    public ShooterSubsystem() {

        // Motor Initialization
        hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorID, MotorType.kBrushless);
        flywheelMotor = new SparkMax(Constants.ShooterConstants.flywheelMotorID, MotorType.kBrushless);
        indexerMotor = new TalonFX(Constants.ShooterConstants.indexerMotorID);
        // Motor Config
        flywheelConfig = new SparkMaxConfig();
            flywheelConfig.idleMode(IdleMode.kBrake);
            flywheelConfig.smartCurrentLimit(50);
            flywheelConfig.secondaryCurrentLimit(60);
            flywheelConfig.voltageCompensation(12);
        flywheelConfig.closedLoop.pid(
            Constants.ShooterConstants.speedkP,
            Constants.ShooterConstants.speedkI,
            Constants.ShooterConstants.speedkD
        );
        hoodConfig = new SparkMaxConfig();
            hoodConfig.idleMode(IdleMode.kBrake);
            hoodConfig.smartCurrentLimit(40);
            hoodConfig.secondaryCurrentLimit(50);
            hoodConfig.voltageCompensation(12);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Feedback Control Initialization        
        speedFeedForward = new SimpleMotorFeedforward(Constants.ShooterConstants.speedkS, Constants.ShooterConstants.speedkV, Constants.ShooterConstants.speedkA);
        desiredSpeed = 0;
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelController = flywheelMotor.getClosedLoopController();
        
        // Hood Config
        hoodEncoder = new DutyCycleEncoder(Constants.ShooterConstants.hoodEncoderChannel);
        hoodController = new PIDController(Constants.ShooterConstants.hoodkP, Constants.ShooterConstants.hoodkI, Constants.ShooterConstants.hoodkD);
        currentAngle = 0;
        desiredAngle = 0;
        indexerSpeed = Constants.ShooterConstants.indexerLoadSpeed;

        // Testing
        SmartDashboard.putNumber("desiredShooterAngle", desiredAngle);
        SmartDashboard.putNumber("desiredFlywheelSpeed", desiredSpeed);
        SmartDashboard.putNumber("indexerSpeed", indexerSpeed);

    }

    public void setDesiredSpeed(double speed) {
         
        flywheelMotor.set(speed);
/**       
        if(speed > 0) {
        double rpm = speed * maxRPM;
        double ff = speedFeedForward.calculate(rpm);
        
        flywheelController.setSetpoint(
            rpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ff,
            ArbFFUnits.kVoltage
        );
        } else {
            flywheelController.setReference(0, ControlType.kDutyCycle); // or stop the motor
        }
        */  
    }
    public void setDebugDesiredSpeed(double speed) {
        if(speed > 0) {
        double rpm = speed * maxRPM;
        double ff = speedFeedForward.calculate(rpm);
        
/**         flywheelController.setSetpoint(
            rpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ff,
            ArbFFUnits.kVoltage
        );*/
        } else {
   //         flywheelController.setReference(0, ControlType.kDutyCycle); // or stop the motor
        }
    }
    public void setHoodAngle(double angle) {
        desiredAngle = angle;
    }
    public void setIndexer(boolean load) {
        if(load){
            indexerMotor.set(indexerSpeed);
        } else {
            indexerMotor.set(0);
        }
    }
    public void reverseIndexer(){
        indexerMotor.set(-indexerSpeed);
    }

    public void periodic(){
        SmartDashboard.putNumber("HoodEncoder", hoodEncoder.get());
        SmartDashboard.putNumber("FlywheelRPM", flywheelEncoder.getVelocity());
        setDebugDesiredSpeed(SmartDashboard.getNumber("desiredFlywheelSpeed", 0));
    }

    public double getCurrentHoodAngle() {
        return currentAngle;
    }

    public boolean atSpeed() {
        double targetRPM = desiredSpeed * maxRPM;
        return Math.abs(flywheelEncoder.getVelocity() - targetRPM) < 100;
    }

}
