package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class MotorConfigUtil {
    private static final int maxAttempts = 3;
    private static final int sparkConfigTimeoutMs = 250;
    private static final List<Alert> failedConfigAlerts = new ArrayList<>();

    public static void configure(TalonFX motor, TalonFXConfiguration config, String name) {
        StatusCode status = motor.getConfigurator().apply(config);
        for (int attempt = 1; attempt < maxAttempts && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config);
        }
        if (!status.isOK()) {
            reportFailure(name, status.toString());
        }
    }

    public static void configure(SparkMax motor, SparkMaxConfig config, String name) {
        motor.setCANTimeout(sparkConfigTimeoutMs);
        REVLibError error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        for (int attempt = 1; attempt < maxAttempts && error != REVLibError.kOk; attempt++) {
            error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        if (error != REVLibError.kOk) {
            reportFailure(name, error.toString());
        }
    }

    private static void reportFailure(String name, String error) {
        Alert alert = new Alert(name + " config failed: " + error, AlertType.kError);
        alert.set(true);
        failedConfigAlerts.add(alert);
    }
}
