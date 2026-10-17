package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MotorConfigUtil;
import frc.robot.generated.TunerConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class ShooterSubsystem extends SubsystemBase{

    // Motors
    SparkMax flywheelMotor;
    TalonFX indexerMotor;
    SparkMaxConfig flywheelConfig;
    TalonFXConfiguration indexerConfig;

    // Flywheel runs open loop, output is -1 to 1
    double desiredSpeed = 0;
    RelativeEncoder flywheelEncoder;
    SparkClosedLoopController flywheelController;
    Debouncer atSpeedDebouncer = new Debouncer(Constants.ShooterConstants.flywheelReadyDebounce, DebounceType.kRising);
    boolean atSpeed = false;
    double spinUpStartTime = 0;
    boolean reachedSpeedSinceSpinUp = false;

    StatusSignal<Current> indexerStatorCurrent;
    StatusSignal<AngularVelocity> indexerVelocity;
    boolean feeding = false;
    boolean unjamming = false;
    double unjamEndTime = 0;
    int jamCount = 0;
    Debouncer jamDebouncer = new Debouncer(Constants.ShooterConstants.indexerJamTime, DebounceType.kRising);

    Alert flywheelSlowAlert = new Alert("Flywheel didn't reach speed within 1 s", AlertType.kWarning);
    Alert flywheelDisconnectedAlert = new Alert("Flywheel SparkMax (CAN 20) not responding", AlertType.kError);
    Alert indexerDisconnectedAlert = new Alert("Indexer TalonFX (CAN 25) not responding", AlertType.kError);
    Debouncer flywheelDisconnectedDebouncer = new Debouncer(0.5);
    Debouncer indexerDisconnectedDebouncer = new Debouncer(0.5);

    SparkMaxSim flywheelSim;
    FlywheelSim flywheelPhysicsSim;
    DCMotorSim indexerPhysicsSim;

    double hoodAngle = Constants.ShooterConstants.scoringHoodAngle;
    double indexerSpeed = 0;

    public ShooterSubsystem() {

        // Motor Initialization
        flywheelMotor = new SparkMax(Constants.ShooterConstants.flywheelMotorID, MotorType.kBrushless);
        indexerMotor = new TalonFX(Constants.ShooterConstants.indexerMotorID, TunerConstants.kCANBus); // indexer is on the DriveTrain CANivore

        // Motor Config
        flywheelConfig = new SparkMaxConfig();
            flywheelConfig.inverted(Constants.ShooterConstants.flywheelInverted);
            flywheelConfig.idleMode(Constants.ShooterConstants.flywheelBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            flywheelConfig.smartCurrentLimit(Constants.ShooterConstants.flywheelCurrentLimit);
            flywheelConfig.secondaryCurrentLimit(Constants.ShooterConstants.flywheelSecondaryCurrentLimit);
            flywheelConfig.voltageCompensation(12); // keeps open loop shots consistent as the battery drains
            if(Constants.ShooterConstants.flywheelClosedLoop) {
                flywheelConfig.closedLoop.pid(Constants.ShooterConstants.flywheelkP, 0, 0);
                flywheelConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.flywheelkV);
            }
        MotorConfigUtil.configure(flywheelMotor, flywheelConfig, "Flywheel");
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelController = flywheelMotor.getClosedLoopController();
        indexerConfig = new TalonFXConfiguration()
                            .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true));
        MotorConfigUtil.configure(indexerMotor, indexerConfig, "Indexer");
        indexerStatorCurrent = indexerMotor.getStatorCurrent();
        indexerVelocity = indexerMotor.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(50, indexerStatorCurrent, indexerVelocity);

        indexerSpeed = Constants.ShooterConstants.indexerLoadSpeed;

        // Testing
        SmartDashboard.putNumber("indexerSpeed", indexerSpeed);

        if(RobotBase.isSimulation()) {
            DCMotor flywheelGearbox = DCMotor.getNEO(1);
            double simGearing = Units.radiansPerSecondToRotationsPerMinute(flywheelGearbox.freeSpeedRadPerSec) / Constants.ShooterConstants.flywheelMaxRPM;
            flywheelSim = new SparkMaxSim(flywheelMotor, flywheelGearbox);
            flywheelPhysicsSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelGearbox, 0.002, simGearing), flywheelGearbox);
            DCMotor indexerGearbox = DCMotor.getKrakenX60(1);
            indexerPhysicsSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(indexerGearbox, 0.001, 1), indexerGearbox);
        }

    }

    /** Open loop flywheel output from -1 to 1. Positive shoots, negative spins the shooter backwards. */
    public void setDesiredSpeed(double speed) {
        double newSpeed = MathUtil.clamp(speed, -1, 1);
        if(newSpeed > 0 && desiredSpeed <= 0) {
            spinUpStartTime = Timer.getFPGATimestamp();
            reachedSpeedSinceSpinUp = false;
        }
        desiredSpeed = newSpeed;
        if(desiredSpeed == 0) {
            flywheelMotor.stopMotor();
        } else if(Constants.ShooterConstants.flywheelClosedLoop) {
            flywheelController.setSetpoint(desiredSpeed * Constants.ShooterConstants.flywheelMaxRPM, ControlType.kVelocity);
        } else {
            flywheelMotor.set(desiredSpeed);
        }
    }
    /** Spin the shooter the opposite direction (unjam). */
    public void reverse() {
        setDesiredSpeed(-Constants.ShooterConstants.flywheelReverseSpeed);
    }
    public void stop() {
        setDesiredSpeed(0);
    }
    @Logged
    public double getDesiredSpeed() {
        return desiredSpeed;
    }
    @Logged
    public double getFlywheelRPM() {
        return flywheelEncoder.getVelocity();
    }
    @Logged
    public boolean isAtSpeed() {
        return atSpeed;
    }
    @Logged
    public double getIndexerCurrent() {
        return indexerStatorCurrent.getValueAsDouble();
    }
    @Logged
    public double getIndexerVelocity() {
        return indexerVelocity.getValueAsDouble();
    }
    @Logged
    public int getJamCount() {
        return jamCount;
    }

    public void setIndexer(boolean load) {
        feeding = load;
        unjamming = false;
        if(load){
            indexerMotor.set(indexerSpeed);
        } else {
            indexerMotor.set(0);
        }
    }
    public void reverseIndexer(){
        feeding = false;
        unjamming = false;
        indexerMotor.set(-indexerSpeed);
    }

    private boolean isIndexerJammed() {
        return Math.abs(getIndexerCurrent()) > Constants.ShooterConstants.indexerJamCurrent
            && Math.abs(getIndexerVelocity()) < Constants.ShooterConstants.indexerJamVelocity;
    }

    private void updateJamClearing() {
        if(!feeding) {
            jamDebouncer.calculate(false);
            return;
        }
        double now = Timer.getFPGATimestamp();
        if(unjamming) {
            if(now >= unjamEndTime) {
                unjamming = false;
                jamDebouncer.calculate(false);
                indexerMotor.set(indexerSpeed);
            }
        } else if(jamDebouncer.calculate(isIndexerJammed())) {
            unjamming = true;
            unjamEndTime = now + Constants.ShooterConstants.indexerUnjamTime;
            jamCount++;
            jamDebouncer.calculate(false);
            indexerMotor.set(-indexerSpeed);
        }
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(indexerStatorCurrent, indexerVelocity);

        double expectedRPM = desiredSpeed * Constants.ShooterConstants.flywheelMaxRPM;
        boolean upToSpeed = desiredSpeed > 0
                && getFlywheelRPM() >= expectedRPM * Constants.ShooterConstants.flywheelReadyFraction;
        atSpeed = atSpeedDebouncer.calculate(upToSpeed);
        if(atSpeed) {
            reachedSpeedSinceSpinUp = true;
        }
        flywheelSlowAlert.set(desiredSpeed > 0 && !reachedSpeedSinceSpinUp
                && Timer.getFPGATimestamp() - spinUpStartTime > Constants.ShooterConstants.spinUpTimeout);

        updateJamClearing();

        flywheelDisconnectedAlert.set(flywheelDisconnectedDebouncer.calculate(flywheelMotor.getBusVoltage() < 4));
        indexerDisconnectedAlert.set(indexerDisconnectedDebouncer.calculate(!indexerMotor.isConnected()));

        SmartDashboard.putNumber("FlywheelOutput", desiredSpeed);
        SmartDashboard.putNumber("FlywheelRPM", getFlywheelRPM());
        SmartDashboard.putBoolean("ShooterReady", atSpeed);
        SmartDashboard.putNumber("JamCount", jamCount);
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        flywheelPhysicsSim.setInputVoltage(flywheelSim.getAppliedOutput() * batteryVoltage);
        flywheelPhysicsSim.update(0.02);
        flywheelSim.iterate(flywheelPhysicsSim.getAngularVelocityRPM(), batteryVoltage, 0.02);

        TalonFXSimState indexerSimState = indexerMotor.getSimState();
        indexerSimState.setSupplyVoltage(batteryVoltage);
        indexerPhysicsSim.setInputVoltage(indexerSimState.getMotorVoltage());
        indexerPhysicsSim.update(0.02);
        indexerSimState.setRawRotorPosition(indexerPhysicsSim.getAngularPositionRotations());
        indexerSimState.setRotorVelocity(Units.radiansToRotations(indexerPhysicsSim.getAngularVelocityRadPerSec()));
    }

    public double getCurrentHoodAngle() {
        return hoodAngle;
    }
}
