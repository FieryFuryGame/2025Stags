package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReef extends Command {

    List<Pose2d> leftPositions = new ArrayList<Pose2d>();
    List<Pose2d> rightPositions = new ArrayList<Pose2d>();
    CommandSwerveDrivetrain drivetrain;
    String trigger;
    Pose2d nearestBranch = new Pose2d();
    boolean finished = false;

    PathConstraints constraints = new PathConstraints(
        5, 4,
        Units.degreesToRadians(1080), Units.degreesToRadians(1648));

    public AlignToReef(CommandSwerveDrivetrain drivetrain, String trigger) {
        this.drivetrain = drivetrain;
        this.trigger = trigger;
        addPosesToArrays();
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        boolean pathIsFine = true;
        Pose2d drivePose = drivetrain.getState().Pose;
        switch (trigger) {
            case "Left":
                nearestBranch = drivePose.nearest(leftPositions);
                break;
            case "Right":
                nearestBranch = drivePose.nearest(rightPositions);
                break;
            default:
                pathIsFine = false;
                break;
        }
        if (pathIsFine) {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivePose, nearestBranch);
            PathPlannerPath generatedPath = new PathPlannerPath(waypoints, constraints, new IdealStartingState((drivetrain.getState().Speeds.vxMetersPerSecond + drivetrain.getState().Speeds.vyMetersPerSecond), drivePose.getRotation()), new GoalEndState(0, nearestBranch.getRotation()));
            AutoBuilder.followPath(generatedPath).schedule();
        }
        finished = true;

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public void addPosesToArrays() {
        leftPositions.add(new Pose2d(3.728, 3.036, Rotation2d.fromDegrees(60))); // 17
        leftPositions.add(new Pose2d(3.225, 4.187, Rotation2d.fromDegrees(0))); // 18
        leftPositions.add(new Pose2d(4.016, 5.194, Rotation2d.fromDegrees(-60))); // 19
        leftPositions.add(new Pose2d(5.263, 5.026, Rotation2d.fromDegrees(-120))); // 20
        leftPositions.add(new Pose2d(5.754, 3.875, Rotation2d.fromDegrees(180))); // 21
        leftPositions.add(new Pose2d(4.987, 2.844, Rotation2d.fromDegrees(120))); // 22

        rightPositions.add(new Pose2d(3.980, 2.868, Rotation2d.fromDegrees(60))); // 17
        rightPositions.add(new Pose2d(3.213, 3.863, Rotation2d.fromDegrees(0))); // 18
        rightPositions.add(new Pose2d(3.728, 5.038, Rotation2d.fromDegrees(-60))); // 19
        rightPositions.add(new Pose2d(4.939, 5.170, Rotation2d.fromDegrees(-120))); // 20
        rightPositions.add(new Pose2d(5.718, 4.199, Rotation2d.fromDegrees(180))); // 21
        rightPositions.add(new Pose2d(5.263, 3.036, Rotation2d.fromDegrees(120))); // 22
    }
}
