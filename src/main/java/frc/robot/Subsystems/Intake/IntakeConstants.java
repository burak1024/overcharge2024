package frc.robot.Subsystems.Intake;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_L_ID = 7;
    public static final int INTAKE_MOTOR_R_ID = 8;
    public static final int ROLLER_MOTOR_L_ID = 9;
    public static final int ROLLER_MOTOR_R_ID = 10;

    public static final double kS = 0.25;
    public static final double kV = 0.119;
    public static final double kA = 0.01;
    public static final double kP = 0.01;
    public static final double kI = 0.01;
    public static final double kD = 0.01;
    public static double[] poses = { 0, 420 / 360 * 2 };
}
