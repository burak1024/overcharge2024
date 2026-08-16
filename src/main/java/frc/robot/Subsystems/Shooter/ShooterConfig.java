package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ShooterConfig {
    public static TalonFXConfiguration config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = ShooterConstants.kS;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kA = ShooterConstants.kA;
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        return config;

    }
}
