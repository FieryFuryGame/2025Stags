package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    private final List<Pose2d> leftPositions = new ArrayList<Pose2d>();
    private final List<Pose2d> rightPositions = new ArrayList<Pose2d>();

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        addPoses();
        SignalLogger.start();
    }

    public void addPoses() {
        leftPositions.add(new Pose2d(3.185, 4.182, Rotation2d.fromDegrees(0)));
        leftPositions.add(new Pose2d(3.690, 3.0, Rotation2d.fromDegrees(60)));
        leftPositions.add(new Pose2d(5.003, 2.795, Rotation2d.fromDegrees(120)));
        leftPositions.add(new Pose2d(5.795, 3.861, Rotation2d.fromDegrees(180)));
        leftPositions.add(new Pose2d(5.290, 5.077, Rotation2d.fromDegrees(-120)));
        leftPositions.add(new Pose2d(3.977, 5.241, Rotation2d.fromDegrees(-60)));
        leftPositions.add(new Pose2d(11.755, 4.189, Rotation2d.fromDegrees(0)));
        leftPositions.add(new Pose2d(12.260, 2.959, Rotation2d.fromDegrees(60)));
        leftPositions.add(new Pose2d(13.573, 2.809, Rotation2d.fromDegrees(120)));
        leftPositions.add(new Pose2d(14.379, 3.847, Rotation2d.fromDegrees(180)));
        leftPositions.add(new Pose2d(13.860, 5.077, Rotation2d.fromDegrees(-120)));
        leftPositions.add(new Pose2d(12.561, 5.241, Rotation2d.fromDegrees(-60)));

        rightPositions.add(new Pose2d(3.185, 3.847, Rotation2d.fromDegrees(0)));
        rightPositions.add(new Pose2d(3.977, 2.809, Rotation2d.fromDegrees(60)));
        rightPositions.add(new Pose2d(5.303, 2.973, Rotation2d.fromDegrees(120)));
        rightPositions.add(new Pose2d(5.795, 4.182, Rotation2d.fromDegrees(180)));
        rightPositions.add(new Pose2d(5.003, 5.255, Rotation2d.fromDegrees(-120)));
        rightPositions.add(new Pose2d(3.677, 5.064, Rotation2d.fromDegrees(-60)));
        rightPositions.add(new Pose2d(11.755, 3.847, Rotation2d.fromDegrees(0)));
        rightPositions.add(new Pose2d(12.561, 2.809, Rotation2d.fromDegrees(60)));
        rightPositions.add(new Pose2d(13.860, 2.986, Rotation2d.fromDegrees(120)));
        rightPositions.add(new Pose2d(14.365, 4.203, Rotation2d.fromDegrees(180)));
        rightPositions.add(new Pose2d(13.559, 5.255, Rotation2d.fromDegrees(-120)));
        rightPositions.add(new Pose2d(12.247, 5.064, Rotation2d.fromDegrees(-60)));
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> nearestLeftTag = driveStateTable.getStructTopic("ClosestLeft", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> nearestRightTag = driveStateTable.getStructTopic("ClosestRight", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        nearestLeftTag.set(state.Pose.nearest(leftPositions));
        nearestRightTag.set(state.Pose.nearest(rightPositions));
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}