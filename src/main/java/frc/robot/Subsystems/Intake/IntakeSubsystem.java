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
    private final TalonFX rollerMotorR = new TalonFX(IntakeConstants.ROLLER_MOTOR_R_ID);
    private final TalonFX rollerMotorL = new TalonFX(IntakeConstants.ROLLER_MOTOR_L_ID);
    private final MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VoltageOut Voltage = new VoltageOut(0).withEnableFOC(true);
    

    public IntakeSubsystem() {
        intakeMotorR.getConfigurator().apply(IntakeConfig.config());
        intakeMotorL.getConfigurator().apply(IntakeConfig.config());
        rollerMotorL.getConfigurator().apply(IntakeConfig.config());
        rollerMotorR.getConfigurator().apply(IntakeConfig.config());

    }

    public Command start(boolean Open, boolean Amphi) {
        return this.runOnce(() -> {
            startIntake(Open, Amphi);
        });
    }

    private void driveIntake(boolean Open) {
        if (Open) {
            intakeMotorL.setControl(MotionMagic.withPosition(IntakeConstants.poses[1]));
            intakeMotorR.setControl(MotionMagic.withPosition(IntakeConstants.poses[1]));
        }
        else {
            intakeMotorL.setControl(MotionMagic.withPosition(IntakeConstants.poses[0]));
            intakeMotorR.setControl(MotionMagic.withPosition(IntakeConstants.poses[0]));
        }

    }

    private Command amphiPick() {
        return Commands.sequence(
                this.run(() -> {
                    intakeMotorL.setControl(MotionMagic.withPosition(IntakeConstants.poses[0]));
                    intakeMotorR.setControl(MotionMagic.withPosition(IntakeConstants.poses[0]));
                }).withTimeout(0.5),
                this.run(() -> {
                    rollerMotorL.setControl(Voltage.withOutput(5.0));
                    rollerMotorR.setControl(Voltage.withOutput(5.0));
                }).withTimeout(1.0));
    }

    private Command amphiShot() {
        return Commands.sequence(
                this.run(() -> {
                    intakeMotorL.setControl(MotionMagic.withPosition(IntakeConstants.poses[1]));
                    intakeMotorL.setControl(MotionMagic.withPosition(IntakeConstants.poses[1]));
                }).withTimeout(1.0),
                this.run(() -> {
                    rollerMotorL.setControl(Voltage.withOutput(-5.0));
                    rollerMotorR.setControl(Voltage.withOutput(-5.0));
                }).withTimeout(1.0)

        );
    }

    private void driveRoller() {
        rollerMotorR.setControl(Voltage.withOutput(3.0));
        rollerMotorL.setControl(Voltage.withOutput(3.0));
    }

    public double getIntakePos() {
        return intakeMotorL.getPosition().getValueAsDouble();
    }

    public void startIntake(boolean Open, boolean Amphi) {
        if (Open && !Amphi) {
            driveIntake(true);
            driveRoller();
        } else if (Open && Amphi)
            amphiPick();
        else if (!Open && Amphi)
            amphiShot();
        else
            stop();

    }

    public void stop() {
        driveIntake(false);
        rollerMotorL.stopMotor();
        rollerMotorR.stopMotor();
    }
}
