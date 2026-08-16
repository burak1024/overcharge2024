package frc.robot;

//import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.TreeMaps.LimelightConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    //private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private ChassisSpeeds speed;
    public Double desiredDegree = 0.0;
    

    public Boolean isReady = false;
    private static CommandSwerveDrivetrain instance;
    
    private final Translation2d target = new Translation2d(0,4.0);
    public final Rotation2d getAngle(){
        Translation2d robotpose = getState().Pose.getTranslation();
        Translation2d desiredDeg = target.minus(robotpose);
        return new Rotation2d(desiredDeg.getX(),desiredDeg.getY());
    }
    public Command turnToAngleCommand() {
        SwerveRequest.FieldCentricFacingAngle rotateRequest =new SwerveRequest.FieldCentricFacingAngle();
        return this.applyRequest(() -> 
        rotateRequest
            .withVelocityX(0.0) 
            .withVelocityY(0.0) 
            .withTargetDirection(getAngle())
    );
}

    private boolean m_hasAppliedOperatorPerspective = false;
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ){
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        for (int i = 0; i < this.getModules().length; i++) {
            TalonFX driveMotor = (TalonFX) this.getModule(i).getDriveMotor();
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
            driveMotor.getConfigurator().refresh(currentLimits);
            currentLimits.withStatorCurrentLimitEnable(true);
            currentLimits.withStatorCurrentLimit(60.0); 
            driveMotor.getConfigurator().apply(currentLimits);
            
        }
        instance=this;
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }
    
    // CommandSwerveDrivetrain.java içinde:
    public Pose2d getPose() {
        return this.getState().Pose;
    }

     private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPoseMG1,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),//p10
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    private void resetPoseMG1(Pose2d pose){
        updateYawMG1();
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }


        updateLimelightOrientation();

        updateVisionForCamera(LimelightConstants.LIMELIGHT_1_NAME);
        updateVisionForCamera(LimelightConstants.LIMELIGHT_2_NAME);

        SmartDashboard.putNumber("distance", target.getDistance(getPose().getTranslation()));
    }

    private void updateLimelightOrientation() {
        double robotYaw = this.getState().Pose.getRotation().getDegrees();
        double robotAngularVelocity = this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation(
            LimelightConstants.LIMELIGHT_1_NAME, 
            robotYaw, 
            robotAngularVelocity, 
            0, 0, 0, 0
        );

        LimelightHelpers.SetRobotOrientation(
            LimelightConstants.LIMELIGHT_2_NAME, 
            robotYaw, 
            robotAngularVelocity, 
            this.getPigeon2().getPitch(true).getValueAsDouble(), 
            0.0,
            this.getPigeon2().getRoll(true).getValueAsDouble(), 
            0.0
        );
    }

    private void updateVisionForCamera(String cameraName) {
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        if(measurement == null) return;
        if (measurement.tagCount == 0 || measurement.pose == null) return;
        addVisionMeasurement(measurement.pose, measurement.timestampSeconds, VecBuilder.fill(0.4, 0.4, 999.0));
    }

    private void updateVisionMG1(String cameraName){
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        if(measurement == null) return;
        if (measurement.tagCount == 0 || measurement.pose == null) return;
        addVisionMeasurement(measurement.pose, measurement.timestampSeconds, VecBuilder.fill(0.25, 1.0, 3.0));
    }

    public void updateYawMG1(){
        updateVisionMG1(LimelightConstants.LIMELIGHT_1_NAME);
        updateVisionMG1(LimelightConstants.LIMELIGHT_2_NAME);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /*public void setTarget(Translation2d target){
        this.target = target;
    }*/

    

    public Double getError(){
        return Math.abs(desiredDegree - getPose().getRotation().getDegrees());
    }

    public ChassisSpeeds getSpeed(){
        speed = super.getState().Speeds;
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            speed.vxMetersPerSecond, 
            speed.vyMetersPerSecond,
            speed.omegaRadiansPerSecond,
            getPose().getRotation().unaryMinus());
    }
    public static CommandSwerveDrivetrain getInstance(){
        return instance;
    }
}