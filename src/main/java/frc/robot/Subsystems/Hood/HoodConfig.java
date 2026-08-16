package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HoodConfig {
    public static TalonFXConfiguration config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = HoodConstants.kS;
        config.Slot0.kV = HoodConstants.kV;
        config.Slot0.kA = HoodConstants.kA;
        config.Slot0.kP = HoodConstants.kP;
        config.Slot0.kI = HoodConstants.kI;
        config.Slot0.kD = HoodConstants.kD;
        return config;
    }
}
