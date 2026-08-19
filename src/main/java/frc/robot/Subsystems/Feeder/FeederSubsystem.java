package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private static final CANrange feederRange = new CANrange(FeederConstants.CANRANGE_ID);
    private static final TalonFX feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);
    private static final VoltageOut Voltage = new VoltageOut(0).withEnableFOC(true);

    public FeederSubsystem() {
        feederMotor.getConfigurator().apply(FeederConfig.config());
    }

    public static void startfeed(boolean open, double voltage) {
        if (open && hasObject()) {
            feederMotor.setControl(Voltage.withOutput(voltage));
        } else
            stop();

    }

    public static boolean hasObject() {
        return feederRange.getIsDetected().getValue();
    }

    public static void stop() {
        feederMotor.stopMotor();
    }
}
