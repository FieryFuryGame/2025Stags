package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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

    List<Pose2d> leftPositions = new ArrayList<Pose2d>();
    List<Pose2d> rightPositions = new ArrayList<Pose2d>();

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
        Pose2d drivePose = drivetrain.getState().Pose;
        Pose2d nearestBranch = new Pose2d();
        trigger = getTriggerPressed(trigger);
        boolean pathIsFine = true;

        addPosesToArrayBlue();
        addPosesToArrayRed();

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
            double currentVelocity = drivetrain.getState().Speeds.vxMetersPerSecond + drivetrain.getState().Speeds.vyMetersPerSecond;
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivePose, nearestBranch);
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, 
                new IdealStartingState(currentVelocity, Rotation2d.fromRadians(drivetrain.getState().Speeds.omegaRadiansPerSecond)), 
                new GoalEndState(0.0, nearestBranch.getRotation())
            );
            path.preventFlipping = true;
            AutoBuilder.followPath(path).schedule();
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

    public void addPosesToArrayRed() {
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

    public void addPosesToArrayBlue() {
        leftPositions.add(new Pose2d(12.262, 3.036, Rotation2d.fromDegrees(60))); // 17
        leftPositions.add(new Pose2d(11.759, 4.187, Rotation2d.fromDegrees(0))); // 18
        leftPositions.add(new Pose2d(12.55, 5.194, Rotation2d.fromDegrees(-60))); // 19
        leftPositions.add(new Pose2d(13.797, 5.026, Rotation2d.fromDegrees(-120))); // 20
        leftPositions.add(new Pose2d(14.288, 3.875, Rotation2d.fromDegrees(180))); // 21
        leftPositions.add(new Pose2d(13.521, 2.844, Rotation2d.fromDegrees(120))); // 22

        rightPositions.add(new Pose2d(12.514, 2.868, Rotation2d.fromDegrees(60))); // 17
        rightPositions.add(new Pose2d(11.747, 3.863, Rotation2d.fromDegrees(0))); // 18
        rightPositions.add(new Pose2d(12.262, 5.038, Rotation2d.fromDegrees(-60))); // 19
        rightPositions.add(new Pose2d(13.473, 5.170, Rotation2d.fromDegrees(-120))); // 20
        rightPositions.add(new Pose2d(14.252, 4.199, Rotation2d.fromDegrees(180))); // 21
        rightPositions.add(new Pose2d(13.797, 3.036, Rotation2d.fromDegrees(120))); // 22
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

}
