package frc.robot.Subsystems.Hood;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TreeMaps;
import frc.robot.Limelight.LimelightConstants;
import frc.robot.Limelight.LimelightHelpers;

public class HoodSubsystem extends SubsystemBase {
    private final static TalonFX hoodRiser = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
    private final static MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0).withEnableFOC(true);

    public HoodSubsystem() {
        hoodRiser.getConfigurator().apply(HoodConfig.config());
    }

    public static void start() {
        setTeamAllience();
        setMeasure();
        setHood();
    }

    private static double setMeasure() {
        Pose2d limelight = LimelightHelpers.getBotPose2d(LimelightConstants.LIMELIGHT_1_NAME);
        double robotx = limelight.getX();
        double measure = HoodConstants.Target.getX() - robotx;
        return Math.abs(measure);

    }
    private static double setPos(){
        return TreeMaps.HoodDegreeMap().get(setMeasure());
    }

    private static void setHood() {
        hoodRiser.setControl(MotionMagic.withPosition(setPos()));
    }
    
    private static double getMotorPos() {
        return hoodRiser.getPosition().getValueAsDouble();
    }

    public static boolean isReady() {
        return getMotorPos() > setPos();
    }

    public static void setTeamAllience() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red)
            HoodConstants.Target = HoodConstants.redTarget;
        else
            HoodConstants.Target = HoodConstants.blueTarget;// attım

    }

}
