package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ClimbConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
    private final MotionMagicVoltage motion = new MotionMagicVoltage(0).withEnableFOC(true);

    public void climb(boolean setPos) {
        if (setPos) {
            climbMotor.setControl(motion.withPosition(1));// kaç olacağını bilemedim
        } else {
            climbMotor.setControl(motion.withPosition(0));
        }
    }

}
