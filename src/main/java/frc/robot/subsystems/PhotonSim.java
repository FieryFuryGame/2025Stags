package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSim extends SubsystemBase {
    
    CommandSwerveDrivetrain drivetrain;

    VisionSystemSim visionSim = new VisionSystemSim("vision");
    PhotonCamera camera = new PhotonCamera("frontcam");
    PhotonCameraSim cameraSim;

    public int tid = 0;

    PathConstraints constraints = new PathConstraints(
        6.0, 5.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    );

    PathPlannerPath alignmentPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));

    public PhotonSim(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
        cameraProp.setFPS(100);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(0), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        cameraSim.enableDrawWireframe(false);

        visionSim.addCamera(cameraSim, robotToCamera);
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
            int id = tid;
            if(id <= 11) {
                PathPlannerPath path = PathPlannerPath.fromPathFile((id + 11) + getTriggerPressed(trigger));
                path.preventFlipping = false;
                System.out.println("[Pathfinder] Pathfinding to the " + trigger + " of AprilTag " + id + "!");
                return path.mirrorPath();
            } else {
                PathPlannerPath path = PathPlannerPath.fromPathFile(id + getTriggerPressed(trigger));
                path.preventFlipping = false;
                System.out.println("[Pathfinder] Pathfinding to the " + trigger + " of AprilTag " + id + "!");
                return path;
            }
        } catch (Exception e) {
            DriverStation.reportError("[Pathfinder] Big oops: " + e.getMessage(), e.getStackTrace());
            waypoints = PathPlannerPath.waypointsFromPoses(
                drivetrain.getState().Pose,
                drivetrain.getState().Pose
            );
            alignmentPath = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(drivetrain.getState().Pose.getRotation().getDegrees())));
            alignmentPath.preventFlipping = true;
            return alignmentPath;
        }
    }

    public Command pathfindWithPath(String trigger) { // Pathfinds to the start of the path, then aligns with the path.
        return AutoBuilder.pathfindThenFollowPath(getPathToTag(trigger), constraints);
    }

    @Override
    public void periodic() {
        visionSim.update(drivetrain.getState().Pose);

        PhotonPipelineResult targets = camera.getLatestResult();

        if (targets.hasTargets()) {
            SmartDashboard.putNumber("result", targets.getBestTarget().fiducialId);
            if (targets.getBestTarget().fiducialId == 6 || targets.getBestTarget().fiducialId == 7 || targets.getBestTarget().fiducialId == 8 || targets.getBestTarget().fiducialId == 9 || targets.getBestTarget().fiducialId == 10 || targets.getBestTarget().fiducialId == 11 || targets.getBestTarget().fiducialId == 17 || targets.getBestTarget().fiducialId == 18 || targets.getBestTarget().fiducialId == 19 || targets.getBestTarget().fiducialId == 20 || targets.getBestTarget().fiducialId == 21 || targets.getBestTarget().fiducialId == 22) {
                tid = targets.getBestTarget().fiducialId;
            } else {
                tid = 0;
            }
        } else {
            tid = 0;
        }
    }

}
