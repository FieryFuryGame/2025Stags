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

public class AlignToAlgae extends Command {
    
    CommandSwerveDrivetrain drivetrain;

    boolean finished = false;
    boolean pathIsFine = true;

    PathConstraints constraints = new PathConstraints(
        3, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public AlignToAlgae(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pathIsFine = true;
        finished = false;
        Pose2d drivePose = drivetrain.getState().Pose;
        Pose2d nearestBranch = new Pose2d();

        List<Pose2d> positions = addPoses();
        nearestBranch = drivePose.nearest(positions);

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

    public List<Pose2d> addPoses() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        poses.add(new Pose2d(3.824, 5.146, Rotation2d.fromDegrees(-60)));
        poses.add(new Pose2d(3.188, 4.015, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(3.832, 2.904, Rotation2d.fromDegrees(60)));
        poses.add(new Pose2d(5.138, 2.894, Rotation2d.fromDegrees(120)));
        poses.add(new Pose2d(5.792, 4.015, Rotation2d.fromDegrees(180)));
        poses.add(new Pose2d(5.138, 5.156, Rotation2d.fromDegrees(-120)));
        

        return poses;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
