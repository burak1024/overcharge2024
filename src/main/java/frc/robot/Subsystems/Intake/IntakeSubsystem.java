package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotorL = new TalonFX(IntakeConstants.INTAKE_MOTOR_L_ID);
    private final TalonFX intakeMotorR = new TalonFX(IntakeConstants.INTAKE_MOTOR_R_ID);
    private final TalonFX RollerMotorR = new TalonFX(IntakeConstants.ROLLER_Motor_R_ID);
    private final TalonFX RollerMotorL = new TalonFX(IntakeConstants.ROLLER_MotorL_ID);
    private final MotionMagicVoltage motion = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VoltageOut voltage = new VoltageOut(0).withEnableFOC(true);
    private Integer[] poses = { 0, 420 / 360 * 2 };

    public IntakeSubsystem() {
        intakeMotorR.getConfigurator().apply(IntakeConfig.config());
        intakeMotorL.getConfigurator().apply(IntakeConfig.config());
        RollerMotorL.getConfigurator().apply(IntakeConfig.config());
        RollerMotorR.getConfigurator().apply(IntakeConfig.config());

    }

    public Command start(boolean isopen, boolean isamphi) {
        return this.runOnce(() -> {
            startIntake(isopen, isamphi);
        });
    }

    private void driveIntake() {
        if (poses[0] == getIntakePos()) {
            intakeMotorL.setControl(motion.withPosition(poses[1]));
            intakeMotorR.setControl(motion.withPosition(poses[1]));
        } else if (poses[0] + 35 > getIntakePos()) {
            intakeMotorL.setControl(motion.withPosition(poses[1]));
            intakeMotorR.setControl(motion.withPosition(poses[1]));
        }

    }

    private Command amphicom() {
        return Commands.sequence(
                this.run(() -> {
                    intakeMotorL.setControl(motion.withPosition(poses[0]));
                    intakeMotorR.setControl(motion.withPosition(poses[0]));
                }).withTimeout(0.5),
                this.run(() -> {
                    RollerMotorL.setControl(voltage.withOutput(5.0));
                    RollerMotorR.setControl(voltage.withOutput(5.0));
                }).withTimeout(1.0));
    }

    private Command amphishot() {
        return Commands.sequence(
                this.run(() -> {
                    intakeMotorL.setControl(motion.withPosition(poses[1]));
                    intakeMotorL.setControl(motion.withPosition(poses[1]));
                }).withTimeout(1.0),
                this.run(() -> {
                    RollerMotorL.setControl(voltage.withOutput(-5.0));
                    RollerMotorR.setControl(voltage.withOutput(-5.0));
                }).withTimeout(1.0)

        );
    }

    private void driveRoller() {
        RollerMotorR.setControl(voltage.withOutput(3.0));
        RollerMotorL.setControl(voltage.withOutput(3.0));
    }

    public double getIntakePos() {
        return intakeMotorL.getPosition().getValueAsDouble();
    }

    public void startIntake(boolean isopen, boolean isamphi) {
        if (isopen && !isamphi) {
            driveIntake();
            driveRoller();
        } else if (isopen && isamphi)
            amphicom();
        else if (!isopen && isamphi)
            amphishot();
        else
            stop();

    }

    public void stop() {
        intakeMotorL.setControl(motion.withPosition(poses[0]));
        intakeMotorR.setControl(motion.withPosition(poses[0]));
        RollerMotorL.stopMotor();
        RollerMotorR.stopMotor();
    }
}
