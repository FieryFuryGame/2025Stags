package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class ExtraDriver extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    public SwerveDrivePoseEstimator m_poseEstimator;

    public PIDController rotationController = new PIDController(0.1, 0, 0);

    public PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentricFacingAngle autoDrive = new FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(15, 0, 0.5);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    List<Pose2d> blueLeftPoses = new ArrayList<Pose2d>();
    List<Pose2d> blueRightPoses = new ArrayList<Pose2d>();
    List<Pose2d> redLeftPoses = new ArrayList<Pose2d>();
    List<Pose2d> redRightPoses = new ArrayList<Pose2d>();

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot 2", Pose2d.struct).publish();

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            Time.ofBaseUnits(30, Seconds), // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public ExtraDriver(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_poseEstimator = new SwerveDrivePoseEstimator
        (getKinematics(), 
        getPigeon2().getRotation2d(), 
        getState().ModulePositions, 
        getState().Pose);

        addLeftPoses();
        addRightPoses();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public ExtraDriver(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        addLeftPoses();
        addRightPoses();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public ExtraDriver(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        addLeftPoses();
        addRightPoses();

    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */

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
        posePublisher.set(getState().Pose);
        SmartDashboard.putNumber("Drive Speed (m/s)", getState().Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Angular Velocity (deg/s)", Math.toDegrees(getState().Speeds.omegaRadiansPerSecond));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    APTarget target = new APTarget(new Pose2d());

    public Pose2d getClosestLeftBranch() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d branchPose = new Pose2d();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                branchPose = getState().Pose.nearest(blueLeftPoses);
            } else if (alliance.get() == Alliance.Blue) {
                branchPose = getState().Pose.nearest(redLeftPoses);
            } else {
                branchPose = new Pose2d();
            }
        }
        return branchPose;
    }

    public Pose2d getClosestRightBranch() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d branchPose = new Pose2d();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                branchPose = getState().Pose.nearest(blueRightPoses);
            } else if (alliance.get() == Alliance.Blue) {
                branchPose = getState().Pose.nearest(redRightPoses);
            } else {
                branchPose = new Pose2d();
            }
        }
        return branchPose;
    }

    public void setTargetToClosestLeftBranch() {
        target = convertPoseToTarget(getClosestLeftBranch());
    }

    public void setTargetToClosestRightBranch() {
        target = convertPoseToTarget(getClosestRightBranch());
    }

    public void stop() {
        setControl(brake);
    }

    public void addLeftPoses() {
        blueLeftPoses.add(new Pose2d(3.185, 4.182, Rotation2d.fromDegrees(0)));
        blueLeftPoses.add(new Pose2d(3.695, 2.982, Rotation2d.fromDegrees(60)));
        blueLeftPoses.add(new Pose2d(5.003, 2.795, Rotation2d.fromDegrees(120)));
        blueLeftPoses.add(new Pose2d(5.795, 3.861, Rotation2d.fromDegrees(180)));
        blueLeftPoses.add(new Pose2d(5.290, 5.077, Rotation2d.fromDegrees(-120)));
        blueLeftPoses.add(new Pose2d(3.977, 5.241, Rotation2d.fromDegrees(-60)));
        redLeftPoses.add(new Pose2d(11.755, 4.189, Rotation2d.fromDegrees(0)));
        redLeftPoses.add(new Pose2d(12.260, 2.959, Rotation2d.fromDegrees(60)));
        redLeftPoses.add(new Pose2d(13.573, 2.809, Rotation2d.fromDegrees(120)));
        redLeftPoses.add(new Pose2d(14.379, 3.847, Rotation2d.fromDegrees(180)));
        redLeftPoses.add(new Pose2d(13.860, 5.077, Rotation2d.fromDegrees(-120)));
        redLeftPoses.add(new Pose2d(12.561, 5.241, Rotation2d.fromDegrees(-60)));
    }

    public void addRightPoses() {
        blueRightPoses.add(new Pose2d(3.185, 3.847, Rotation2d.fromDegrees(0)));
        blueRightPoses.add(new Pose2d(3.977, 2.809, Rotation2d.fromDegrees(60)));
        blueRightPoses.add(new Pose2d(5.303, 2.973, Rotation2d.fromDegrees(120)));
        blueRightPoses.add(new Pose2d(5.795, 4.182, Rotation2d.fromDegrees(180)));
        blueRightPoses.add(new Pose2d(5.003, 5.255, Rotation2d.fromDegrees(-120)));
        blueRightPoses.add(new Pose2d(3.677, 5.064, Rotation2d.fromDegrees(-60)));
        redRightPoses.add(new Pose2d(11.755, 3.847, Rotation2d.fromDegrees(0)));
        redRightPoses.add(new Pose2d(12.561, 2.809, Rotation2d.fromDegrees(60)));
        redRightPoses.add(new Pose2d(13.860, 2.986, Rotation2d.fromDegrees(120)));
        redRightPoses.add(new Pose2d(14.365, 4.203, Rotation2d.fromDegrees(180)));
        redRightPoses.add(new Pose2d(13.572, 5.244, Rotation2d.fromDegrees(-120)));
        redRightPoses.add(new Pose2d(12.247, 5.064, Rotation2d.fromDegrees(-60)));
    }

    public APTarget convertPoseToTarget(Pose2d pose) {
        APTarget target = new APTarget(pose).withEntryAngle(pose.getRotation());
        return target;
    }

    public Command alignmentCommand = run(() -> {
        Translation2d velocity = new Translation2d(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond).rotateBy(getState().Pose.getRotation());
        Pose2d pose2d = getState().Pose;

        Transform2d output = Constants.AutopilotConstants.autopilot.calculate(pose2d, velocity, target);

        double x = output.getX();
        double y = output.getY();
        Rotation2d heading = output.getRotation();

        setControl(autoDrive
            .withVelocityX(x)
            .withVelocityY(y)
            .withTargetDirection(heading)
        );
    }).until(() -> Constants.AutopilotConstants.autopilot.atTarget(getState().Pose, target))
    .finallyDo(this::stop);
}