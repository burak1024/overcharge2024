package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeConfig {
    public static TalonFXConfiguration config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = IntakeConstants.kS;
        config.Slot0.kV = IntakeConstants.kV;
        config.Slot0.kA = IntakeConstants.kA;
        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;
        return config;
    }
}
