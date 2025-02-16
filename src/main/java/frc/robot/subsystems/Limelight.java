package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;

public class Limelight extends SubsystemBase {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    String name;
    NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    NetworkTable table;

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon2;
    
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;
    PoseEstimate visionPoseEstimate;
    double ThreeDRotationMeasurement;
    double ThreeDDistanceMeasurement;
    public int teamAdd = 0;
    public int rotateDirection = 0;
    int tagAngle;
    Pose2d targetPose;
    List<Waypoint> waypoints;

    InterpolatingDoubleTreeMap areaMap = new InterpolatingDoubleTreeMap();

    public void setupMap() {
        areaMap.put(41.0, 8.25);
        areaMap.put(17.5, 14.0);
        areaMap.put(6.6, 24.0);
        areaMap.put(5.4, 26.625);
        areaMap.put(4.1, 31.125);
        areaMap.put(3.2, 35.375);
        areaMap.put(2.37, 41.5);
        areaMap.put(1.6, 50.375);
        areaMap.put(0.98, 65.625);
        areaMap.put(0.64, 81.3125);
        areaMap.put(0.3, 114.125);
    }

    public Limelight(String limelightName, CommandSwerveDrivetrain swerve) {
        setupMap();
        name = limelightName;
        drivetrain = swerve;
        pigeon2 = drivetrain.getPigeon2();
        table = tableInstance.getTable(limelightName);
        targetPose = drivetrain.getState().Pose;
    }

    public double getTargetDistanceMath() {

        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0.343;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 8;

        // distance from the target to the floor
        double goalHeightInches = 12.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        
        return distanceFromLimelightToGoalInches;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        0.5, 0.2,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public List<Waypoint> createWaypoints() {
        return PathPlannerPath.waypointsFromPoses(
            drivetrain.getState().Pose,
            targetPose
        );
    }

    public PathPlannerPath generatePath() {
        return new PathPlannerPath(
            createWaypoints(), 
            constraints, 
            null, 
            new GoalEndState(0.0, targetPose.getRotation()));
    }

    public Command runPath() {
        return AutoBuilder.followPath(generatePath());
    }

    public Command pathfind() {
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return Commands.runOnce(() -> Pathfinding.setStartPosition(new Translation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY()))).andThen(
            AutoBuilder.pathfindToPoseFlipped(
                Constants.AlignmentConstants.I_BLUE,
                constraints,
                0.0 // Goal end velocity in meters/sec
            )
        );
    }
    public Command setPathfindPose() {
        return Commands.runOnce(() -> {
            switch ((int)tid.getDouble(0.0)) {
            case 18, 7:
                targetPose = Constants.AlignmentConstants.A_BLUE;
                System.out.println("A");
                break;
            case 19, 6:
                targetPose = Constants.AlignmentConstants.K_BLUE;
                System.out.println("B");
                break;
            case 20,  11:
                targetPose = Constants.AlignmentConstants.I_BLUE;
                System.out.println("C");
                break;
            case 21, 10:
                targetPose = Constants.AlignmentConstants.G_BLUE;
                System.out.println("D");
                break;
            case 22, 9:
                targetPose = Constants.AlignmentConstants.E_BLUE;
                System.out.println("E");
                break;
            case 17, 8:
                targetPose = Constants.AlignmentConstants.C_BLUE;
                System.out.println("F");
                break;
            default:
                System.out.println("I think there might be a problem.");
                targetPose = drivetrain.getState().Pose;
                break;
            }
        });
            
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(name, MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees(), 0, 360), 0, MathUtil.inputModulus(pigeon2.getPitch().getValueAsDouble(), 0, 360), 0, MathUtil.inputModulus(pigeon2.getRoll().getValueAsDouble(), 0, 360), 0);
        
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tid = table.getEntry("tid");
        ta = table.getEntry("ta");
        visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        
        SmartDashboard.putNumber("tr", MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees() + teamAdd, 0,360));
        SmartDashboard.putNumber("td", areaMap.get(ta.getDouble(0.0))); // Target Distance

        
    }
}
