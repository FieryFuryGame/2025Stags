package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    String name;
    NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    NetworkTable table;

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon2;
    
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    public NetworkTableEntry tid;
    PoseEstimate visionPoseEstimate;
    double ThreeDRotationMeasurement;
    double ThreeDDistanceMeasurement;
    public int teamAdd = 0;
    public int rotateDirection = 0;
    public int override = 6;
    int tagAngle;
    Pose2d targetPose = new Pose2d();

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        4.0, 3.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    );
    PathPlannerPath alignmentPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));

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

    public String getTriggerPressed(String trigger) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            switch (trigger) {
                case "Left":
                    trigger = "Right";
                    break;
                case "Right":
                    trigger = "Left";
                    break;
            }
            return trigger;
        } else {
            return trigger;
        }
    }

    public PathPlannerPath getPathToTag(String trigger) { // Uses the tag id and the name to find the path file we've created.
        try{
            if(override <= 11) {
                PathPlannerPath path = PathPlannerPath.fromPathFile((override + 11) + getTriggerPressed(trigger));
                path.preventFlipping = false;
                System.out.println("[Pathfinder] Pathfinding to the " + trigger + " of AprilTag!");
                return path.mirrorPath();
            } else {
                PathPlannerPath path = PathPlannerPath.fromPathFile(override + getTriggerPressed(trigger));
                path.preventFlipping = false;
                System.out.println("[Pathfinder] Pathfinding to the " + trigger + " of AprilTag!");
                return path;
            }
        } catch (Exception e) {
            DriverStation.reportError("[Pathfinder] Big oops: " + e.getMessage(), e.getStackTrace());
            waypoints = PathPlannerPath.waypointsFromPoses(
                drivetrain.getState().Pose,
                drivetrain.getState().Pose
            );
            alignmentPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(drivetrain.getState().Pose.getRotation().getDegrees())));
            return alignmentPath;
        }
    }

    public Command pathfindWithPath(String trigger) { // Pathfinds to the start of the path, then aligns with the path.
        return AutoBuilder.pathfindThenFollowPath(getPathToTag(trigger), constraints);
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(name, MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees(), 0, 360), 0, MathUtil.inputModulus(pigeon2.getPitch().getValueAsDouble(), 0, 360), 0, MathUtil.inputModulus(pigeon2.getRoll().getValueAsDouble(), 0, 360), 0);
        
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
        visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        
        
        SmartDashboard.putNumber("tr", MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees() + teamAdd, 0,360));
        SmartDashboard.putNumber("td", areaMap.get(ta.getDouble(0.0))); // Target Distance
        SmartDashboard.putNumber("overrideTid", override);

        
    }
}
