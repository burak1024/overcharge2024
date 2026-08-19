package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Hood.HoodSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TreeMaps;

public class ShooterSubsystem extends SubsystemBase {
    public static CommandSwerveDrivetrain drivetrain;
    private final static TalonFX shooterMotorR = new TalonFX(ShooterConstants.SHOOTER_MOTOR_R_ID);
    private final static TalonFX shooterMotorL = new TalonFX(ShooterConstants.SHOOTER_MOTOR_L_ID);
    private final static VoltageOut voltage = new VoltageOut(0).withEnableFOC(true);

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        ShooterSubsystem.drivetrain = drivetrain;
        shooterMotorL.getConfigurator().apply(ShooterConfig.config());
        shooterMotorR.getConfigurator().apply(ShooterConfig.config());
    }
    public Command start(boolean open) {
        return this.runOnce(() -> {
            startShoot(open);
            FeederSubsystem.startfeed(open, setShooterVoltage());
            HoodSubsystem.start();

        });
    
    }

    public static void startShoot(boolean ShootMod) {
        if (ShootMod && FeederSubsystem.hasObject() && HoodSubsystem.isReady()) {
            shooterMotorL.setControl(voltage.withOutput(setShooterVoltage()));
            shooterMotorR.setControl(voltage.withOutput(setShooterVoltage()));
        }

    }

    private static double setShooterVoltage() {
        Pose2d robotpos = drivetrain.getPose();
        Translation2d target = new Translation2d(0.0, 4.0);
        double measure = target.getX() - robotpos.getX();
        return TreeMaps.ShooterVoltageMap().get(measure);
    }

}
