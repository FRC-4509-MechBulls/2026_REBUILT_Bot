package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MotorConfigUtil;


@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeSubsystem extends SubsystemBase{

    // Motors
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMaxConfig sparkMaxConfig;
    TalonFX intakeMotor;
    TalonFXConfiguration talonFXConfiguration;

    // Feedback Control
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    double requestedHopperVolts;
    double extendedReference = Double.NaN;
    double retractedReference = 0;

    double intakeWheelSpeed;
    double intakeWheelOutput;

    Alert leftMotorDisconnectedAlert = new Alert("Hopper left SparkMax (CAN 19) not responding", AlertType.kError);
    Alert rightMotorDisconnectedAlert = new Alert("Hopper right SparkMax (CAN 30) not responding", AlertType.kError);
    Alert intakeMotorDisconnectedAlert = new Alert("Intake roller TalonFX (CAN 21) not responding", AlertType.kError);
    Debouncer leftMotorDisconnectedDebouncer = new Debouncer(0.5);
    Debouncer rightMotorDisconnectedDebouncer = new Debouncer(0.5);
    Debouncer intakeMotorDisconnectedDebouncer = new Debouncer(0.5);

    SparkMaxSim leftMotorSim;
    SparkMaxSim rightMotorSim;
    DCMotorSim hopperPhysicsSim;
    DCMotorSim intakePhysicsSim;

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
        MotorConfigUtil.configure(leftMotor, sparkMaxConfig, "Hopper left");
        MotorConfigUtil.configure(rightMotor, sparkMaxConfig, "Hopper right");

        talonFXConfiguration = new TalonFXConfiguration()
                                    .withCurrentLimits(new CurrentLimitsConfigs()
                                                        .withStatorCurrentLimit(40)
                                                        .withSupplyCurrentLimit(40)
                                    );
        MotorConfigUtil.configure(intakeMotor, talonFXConfiguration, "Intake roller");

        // Feedback Control Initialization
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        requestedHopperVolts = 0;

        intakeWheelSpeed = Constants.IntakeConstants.intakeWheelSpeed;
        intakeWheelOutput = 0;

        if(RobotBase.isSimulation()) {
            DCMotor hopperGearbox = DCMotor.getNEO(2);
            leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
            rightMotorSim = new SparkMaxSim(rightMotor, DCMotor.getNEO(1));
            hopperPhysicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(hopperGearbox, 0.01, 1), hopperGearbox);
            DCMotor intakeGearbox = DCMotor.getKrakenX60(1);
            intakePhysicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeGearbox, 0.001, 1), intakeGearbox);
        }

    }

    @Override
    public void periodic(){

        applyHopperVoltage();
        updateHopperReferences();

        leftMotorDisconnectedAlert.set(leftMotorDisconnectedDebouncer.calculate(leftMotor.getBusVoltage() < 4));
        rightMotorDisconnectedAlert.set(rightMotorDisconnectedDebouncer.calculate(rightMotor.getBusVoltage() < 4));
        intakeMotorDisconnectedAlert.set(intakeMotorDisconnectedDebouncer.calculate(!intakeMotor.isConnected()));

        SmartDashboard.putNumber("HopperPosition", getHopperPosition());

    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        double hopperTravel = Constants.IntakeConstants.hopperExtendedRotations > 0 ? Constants.IntakeConstants.hopperExtendedRotations : 40;

        hopperPhysicsSim.setInputVoltage(leftMotorSim.getAppliedOutput() * batteryVoltage);
        hopperPhysicsSim.update(0.02);
        double hopperPosition = hopperPhysicsSim.getAngularPositionRotations();
        if(hopperPosition < 0 || hopperPosition > hopperTravel) {
            hopperPhysicsSim.setState(Units.rotationsToRadians(MathUtil.clamp(hopperPosition, 0, hopperTravel)), 0);
        }
        double hopperRPM = hopperPhysicsSim.getAngularVelocityRPM();
        leftMotorSim.iterate(hopperRPM, batteryVoltage, 0.02);
        rightMotorSim.iterate(-hopperRPM, batteryVoltage, 0.02);

        TalonFXSimState intakeSimState = intakeMotor.getSimState();
        intakeSimState.setSupplyVoltage(batteryVoltage);
        intakePhysicsSim.setInputVoltage(intakeSimState.getMotorVoltage());
        intakePhysicsSim.update(0.02);
        intakeSimState.setRawRotorPosition(intakePhysicsSim.getAngularPositionRotations());
        intakeSimState.setRotorVelocity(Units.radiansToRotations(intakePhysicsSim.getAngularVelocityRadPerSec()));
    }

    public void controlIntake(double leftY, double rightY){
        setIntakeWheels(rightY);
        setMotors(leftY*6);
    }

    public void setMotors(double speed){
        requestedHopperVolts = speed;
        applyHopperVoltage();
    }

    private void applyHopperVoltage() {
        double volts = requestedHopperVolts;

        if(Constants.IntakeConstants.hopperLimitsEnabled) {
            boolean pastExtended = volts < 0 && getHopperPosition() >= Constants.IntakeConstants.hopperExtendedRotations;
            boolean pastRetracted = volts > 0 && getHopperPosition() <= Constants.IntakeConstants.hopperRetractedRotations;
            if(pastExtended || pastRetracted) {
                volts = 0;
            }
        }

        leftMotor.setVoltage(-volts);
        rightMotor.setVoltage(volts);
    }

    @Logged
    public double getHopperPosition() {
        return (leftEncoder.getPosition() - rightEncoder.getPosition()) / 2;
    }

    @Logged
    public double getRequestedHopperVolts() {
        return requestedHopperVolts;
    }

    @Logged
    public boolean isHopperExtended() {
        if(Constants.IntakeConstants.hopperLimitsEnabled) {
            return isHopperAtExtendedLimit();
        }
        if(Double.isNaN(extendedReference)) {
            return false;
        }
        double position = getHopperPosition();
        return Math.abs(position - extendedReference) < Math.abs(position - retractedReference);
    }

    public boolean isHopperAtExtendedLimit() {
        return Constants.IntakeConstants.hopperLimitsEnabled
            && getHopperPosition() >= Constants.IntakeConstants.hopperExtendedRotations - Constants.IntakeConstants.hopperPositionTolerance;
    }

    public boolean isHopperAtRetractedLimit() {
        return Constants.IntakeConstants.hopperLimitsEnabled
            && getHopperPosition() <= Constants.IntakeConstants.hopperRetractedRotations + Constants.IntakeConstants.hopperPositionTolerance;
    }

    private void updateHopperReferences() {
        if(Double.isNaN(extendedReference)) {
            return;
        }
        double position = getHopperPosition();
        double extendDirection = Math.signum(extendedReference - retractedReference);
        if((position - extendedReference) * extendDirection > 0) {
            extendedReference = position;
        }
        if((retractedReference - position) * extendDirection > 0) {
            retractedReference = position;
        }
    }

    public void markHopperExtended() {
        extendedReference = getHopperPosition();
    }

    public void markHopperRetracted() {
        retractedReference = getHopperPosition();
    }

    public void zeroHopper() {
        double position = getHopperPosition();
        if(!Double.isNaN(extendedReference)) {
            extendedReference -= position;
        }
        retractedReference = 0;
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void intake(boolean intake){
        if(intake){
            setIntakeWheels(intakeWheelSpeed);
        } else {
            setIntakeWheels(0);
        }
    }

    private void setIntakeWheels(double output) {
        intakeWheelOutput = output;
        intakeMotor.set(output);
    }

    @Logged
    public double getIntakeWheelOutput() {
        return intakeWheelOutput;
    }

    @Logged
    public boolean isIntaking() {
        return Math.abs(intakeWheelOutput) > 0.05;
    }

}
