package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Tuner X tarafından oluşturulan sabitler ve subsystem
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    // 1. Sürüş Hız Sınırları
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // 2. Swerve İstekleri (SwerveRequest)
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    // 3. Subsystem ve CommandXboxController Tanımları
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // Continuous Swerve Sürüşü (Default Command)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveRequest
                    .withVelocityX(-driverController.getLeftY() * maxSpeed)
                    .withVelocityY(-driverController.getLeftX() * maxSpeed)
                    .withRotationalRate(-driverController.getRightX() * maxAngularRate)
            )
        );
    }

    private void configureButtonBindings() {
        // Y Butonuna basılınca Gyro / Saha Yönünü Sıfırla
        driverController.y().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // X Butonuna basılı tutulunca Tekerlekleri Kilitlesin (Brake)
        driverController.x().whileTrue(
            drivetrain.applyRequest(() -> brakeRequest)
        );

        // Kendi ekstra motor/komut bağlantılarınızı buraya yazacaksınız:
        // driverController.a().whileTrue(intakeSubsystem.runIntakeCommand());
    }

    public Command getAutonomousCommand() {
        // Otonom komutunuzu buradan dönebilirsiniz
        return null;
    }
}