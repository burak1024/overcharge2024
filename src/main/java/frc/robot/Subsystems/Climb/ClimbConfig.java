package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ClimbConfig {
    public static TalonFXConfiguration config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = ClimbConstants.kS;
        config.Slot0.kV = ClimbConstants.kV;
        config.Slot0.kA = ClimbConstants.kA;
        config.Slot0.kP = ClimbConstants.kP;
        config.Slot0.kI = ClimbConstants.kI;
        config.Slot0.kD = ClimbConstants.kD;
        return config;
    }
}
