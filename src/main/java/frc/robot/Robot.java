package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Climb.ClimbSubsystem;
import frc.robot.Subsystems.Hood.HoodSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    public CommandSwerveDrivetrain drivetrain =new CommandSwerveDrivetrain(null, null);
    public ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);
    private static final CommandXboxController driver = new CommandXboxController(0);
    public IntakeSubsystem intake = new IntakeSubsystem();
    public HoodSubsystem hood = new HoodSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();
    public static CommandXboxController getDriver() {
        return driver;
    }

    public Robot() {
        driver.a().onTrue(shooter.start(true));
        driver.rightTrigger().whileTrue(drivetrain.turnToAngleCommand());
        driver.b().onTrue(climb.setClimb(true)).onFalse(climb.setClimb(false));
        driver.x().onTrue(intake.start(true,true )).onFalse(intake.start(false, true));
        driver.leftTrigger().onTrue(intake.start(true,false)).onFalse(intake.start(false, false));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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

    }

    @Override
    public void robotInit() {
    }

}
