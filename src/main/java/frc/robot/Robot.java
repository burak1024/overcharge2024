package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.Subsystems.Climb.ClimbSubsystem;
import frc.robot.Subsystems.Hood.HoodSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class Robot extends TimedRobot {
    private final double MaxSpeed = 5.0;
    private final double MaxAngularRate = 1.5 * Math.PI;
    private Command m_autonomousCommand;
    public CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);
    private static final CommandXboxController driver = new CommandXboxController(0);
    public IntakeSubsystem intake = new IntakeSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();
    
    public static CommandXboxController getDriver() {
        return driver;
    }

    public static final CommandPS4Controller dualshock = new CommandPS4Controller(0);
    PathPlannerPath path;
    public Robot() {
        try {
        path = PathPlannerPath.fromPathFile("path");
    } catch (Exception e) {
        System.out.println("Rota dosyası bulunamadı veya okunamadı!");
        e.printStackTrace();
    }
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
                        .withVelocityX(-dualshock.getLeftY() * MaxSpeed)
                        .withVelocityY(-dualshock.getLeftX() * MaxSpeed)
                        .withRotationalRate(MathUtil.applyDeadband(-dualshock.getRightX(), 0.5) * MaxAngularRate)

                ));
        driver.a().onTrue(shooter.start(true));
        driver.rightTrigger().whileTrue(drivetrain.turnToAngleCommand());
        driver.b().onTrue(climb.setClimb(true)).onFalse(climb.setClimb(false));
        driver.x().onTrue(intake.start(true, true)).onFalse(intake.start(false, true));
        driver.leftTrigger().onTrue(intake.start(true, false)).onFalse(intake.start(false, false));

        dualshock.triangle().whileTrue(shooter.start(true));
        dualshock.R2().whileTrue(drivetrain.turnToAngleCommand());
        dualshock.circle().whileTrue(climb.setClimb(true)).onFalse(climb.setClimb(false));
        dualshock.cross().whileTrue(intake.start(true, true)).onFalse(intake.start(false, true));
        dualshock.L2().whileTrue(intake.start(true, false)).whileFalse(intake.start(false, false));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Pose2d pose = drivetrain.getState().Pose;
        SmartDashboard.putNumberArray("robotpose", new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });
    }

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    public void robotInit() {
    }

}
