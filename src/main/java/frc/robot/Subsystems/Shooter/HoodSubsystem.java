package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final TalonFX hoodMotorR = new TalonFX(0);
    private final TalonFX hoodMotorL = new TalonFX(0);
    private final TalonFX hoodRiser = new TalonFX(0);
    private final VoltageOut voltage = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage motion = new MotionMagicVoltage(0).withEnableFOC(true);

    public HoodSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void start(boolean open) {
        startShoot(open);
        FeederSubsystem.startfeed(open, sethoodVoltage());
    }

    public void startShoot(boolean shootmod) {
        if (shootmod && FeederSubsystem.hasObject()) {
            sethood();
            hoodMotorL.setControl(voltage.withOutput(5.0));
            hoodMotorR.setControl(voltage.withOutput(5.0));
        }

    }

    private double sethoodVoltage() {
        Pose2d robotpos = drivetrain.getPose();
        Translation2d target = new Translation2d(0.0, 4.0);
        double measure = target.getX() - robotpos.getX();
        return Constants.shooterxMap().get(measure);
    }

    private void sethood() {
        double x = 0;
        double robotx = drivetrain.getPose().getX();
        double measure = x - robotx;
        double abs = Math.abs(measure);
        hoodRiser.setControl(motion.withPosition(Constants.hooddegMap().get(abs)));
    }

    
}
