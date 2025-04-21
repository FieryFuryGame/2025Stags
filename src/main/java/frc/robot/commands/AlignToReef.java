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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReef extends Command {
    
    CommandSwerveDrivetrain drivetrain;
    String trigger;

    boolean finished = false;
    boolean pathIsFine = true;

    PathConstraints constraints = new PathConstraints(
        5.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public AlignToReef(CommandSwerveDrivetrain drivetrain, String trigger) {
        this.drivetrain = drivetrain;
        this.trigger = trigger;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pathIsFine = true;
        finished = false;
        Pose2d drivePose = drivetrain.getState().Pose;
        Pose2d nearestBranch = new Pose2d();

        List<Pose2d> leftPositions = addLeftPoses();
        List<Pose2d> rightPositions = addRightPoses();
        if (pathIsFine) {
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
        }

        if (pathIsFine) {
            double currentVelocity = drivetrain.getState().Speeds.vxMetersPerSecond + drivetrain.getState().Speeds.vyMetersPerSecond;
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivePose, nearestBranch);
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, 
                new IdealStartingState(currentVelocity, Rotation2d.fromRadians(drivetrain.getState().Speeds.omegaRadiansPerSecond)), 
                new GoalEndState(0.0, nearestBranch.getRotation())
            );
            path.preventFlipping = true;
            AutoBuilder.followPath(path).schedule();
        } else {
            System.out.println("[Pathfinding] Something went wrong. Cancelling...");
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

    public List<Pose2d> addLeftPoses() {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            poses.add(new Pose2d(3.185, 4.182, Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.690, 3.0, Rotation2d.fromDegrees(60)));
            poses.add(new Pose2d(5.003, 2.795, Rotation2d.fromDegrees(120)));
            poses.add(new Pose2d(5.795, 3.861, Rotation2d.fromDegrees(180)));
            poses.add(new Pose2d(5.290, 5.077, Rotation2d.fromDegrees(-120)));
            poses.add(new Pose2d(3.977, 5.241, Rotation2d.fromDegrees(-60)));
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            poses.add(new Pose2d(11.755, 4.189, Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(12.260, 2.959, Rotation2d.fromDegrees(60)));
            poses.add(new Pose2d(13.573, 2.809, Rotation2d.fromDegrees(120)));
            poses.add(new Pose2d(14.379, 3.847, Rotation2d.fromDegrees(180)));
            poses.add(new Pose2d(13.860, 5.077, Rotation2d.fromDegrees(-120)));
            poses.add(new Pose2d(12.561, 5.241, Rotation2d.fromDegrees(-60)));
        } else {
            pathIsFine = false;
        }

        return poses;
    }

    public List<Pose2d> addRightPoses() {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            poses.add(new Pose2d(3.185, 3.847, Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.977, 2.809, Rotation2d.fromDegrees(60)));
            poses.add(new Pose2d(5.303, 2.973, Rotation2d.fromDegrees(120)));
            poses.add(new Pose2d(5.795, 4.182, Rotation2d.fromDegrees(180)));
            poses.add(new Pose2d(5.003, 5.255, Rotation2d.fromDegrees(-120)));
            poses.add(new Pose2d(3.677, 5.064, Rotation2d.fromDegrees(-60)));
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            poses.add(new Pose2d(11.755, 3.847, Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(12.561, 2.809, Rotation2d.fromDegrees(60)));
            poses.add(new Pose2d(13.860, 2.986, Rotation2d.fromDegrees(120)));
            poses.add(new Pose2d(14.365, 4.203, Rotation2d.fromDegrees(180)));
            poses.add(new Pose2d(13.559, 5.255, Rotation2d.fromDegrees(-120)));
            poses.add(new Pose2d(12.247, 5.064, Rotation2d.fromDegrees(-60)));
        } else {
            pathIsFine = false;
        }

        return poses;
    }

}
