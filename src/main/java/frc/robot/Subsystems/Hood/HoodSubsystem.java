package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class HoodSubsystem extends SubsystemBase {
    private final static TalonFX hoodRiser = new TalonFX(0);
    private final static MotionMagicVoltage motion = new MotionMagicVoltage(0).withEnableFOC(true);
    private static final Pose2d limelight = LimelightHelpers.getBotPose2d("limelight");
    private static double abs = 0;
    private static double x = 0;
    public HoodSubsystem() {

    }

    public static void start() {
        sethood();
    }

    private static double setmeasure() {
        double robotx = limelight.getX();
        double measure = x - robotx;
        return Math.abs(measure);

    }

    private static void sethood() {
        if (!isReady())
        hoodRiser.setControl(motion.withPosition(Constants.hooddegMap().get(setmeasure())));
    }

    public static double getMotorPos() {
        return hoodRiser.getPosition().getValueAsDouble();
    }

    public static boolean isReady() {
        return getMotorPos() > Constants.hooddegMap().get(abs) - 2.0;
    }
    public static void setTeamAllience(){
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance==Alliance.Red)x=0;
        else x=18;//attım
    
    }

}
