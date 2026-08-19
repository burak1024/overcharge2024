package frc.robot.Subsystems.Hood;

import edu.wpi.first.math.geometry.Pose2d;

public class HoodConstants {
    public static final int HOOD_MOTOR_ID = 11;
    public static final double kS = 0.25;
    public static final double kV = 0.119;
    public static final double kA = 0.01;
    public static final double kP = 0.01;
    public static final double kI = 0.01;
    public static final double kD = 0.01;
    public static Pose2d Target = new Pose2d(0,0, null);
    public static Pose2d redTarget = new Pose2d(0,4,null);
    public static Pose2d blueTarget =new Pose2d(18,4,null);
}
