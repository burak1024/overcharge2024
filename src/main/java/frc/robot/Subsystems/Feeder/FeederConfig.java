package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class FeederConfig {
    public static TalonFXConfiguration config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = FeederConstants.kS;
        config.Slot0.kV = FeederConstants.kV;
        config.Slot0.kA = FeederConstants.kA;
        config.Slot0.kP = FeederConstants.kP;
        config.Slot0.kI = FeederConstants.kI;
        config.Slot0.kD = FeederConstants.kD;
        return config;
    }
}