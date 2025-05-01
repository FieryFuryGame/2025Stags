// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSimExtra;
import frc.robot.subsystems.ExtraDriver;

// import frc.robot.subsystems.CANLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class SimulatePlacingCoralL2Extra extends Command {
  ExtraDriver drivetrain;
  EndEffector effector;
  EndEffectorSimExtra effectorSim;

  Pose3d coralPose = new Pose3d();
  List<Pose2d> branchRobotPositions = new ArrayList<Pose2d>();
  Pose2d nearestPose = new Pose2d();
  boolean canPlace = false;
  int branchID = 2;

  public SimulatePlacingCoralL2Extra(ExtraDriver drivetrain, EndEffector effector, EndEffectorSimExtra effectorSim) {
    this.drivetrain = drivetrain;
    this.effector = effector;
    this.effectorSim = effectorSim;

    branchRobotPositions.add(new Pose2d(3.95, 4.63, Rotation2d.fromDegrees(60)));
    branchRobotPositions.add(new Pose2d(4.232, 4.78, Rotation2d.fromDegrees(60)));
    branchRobotPositions.add(new Pose2d(4.735, 4.810, Rotation2d.fromDegrees(0)));
    branchRobotPositions.add(new Pose2d(5.023, 4.631, Rotation2d.fromDegrees(0)));
    branchRobotPositions.add(new Pose2d(5.3, 4.2, Rotation2d.fromDegrees(-60)));
    branchRobotPositions.add(new Pose2d(5.3, 3.875, Rotation2d.fromDegrees(-60)));
    branchRobotPositions.add(new Pose2d(5.035, 3.396, Rotation2d.fromDegrees(-120)));
    branchRobotPositions.add(new Pose2d(4.759, 3.264, Rotation2d.fromDegrees(-120)));
    branchRobotPositions.add(new Pose2d(4.220, 3.228, Rotation2d.fromDegrees(180)));
    branchRobotPositions.add(new Pose2d(3.908, 3.396, Rotation2d.fromDegrees(180)));
    branchRobotPositions.add(new Pose2d(3.7, 3.863, Rotation2d.fromDegrees(120)));
    branchRobotPositions.add(new Pose2d(3.7, 4.2, Rotation2d.fromDegrees(120)));

    branchRobotPositions.add(new Pose2d(12.544, 4.63, Rotation2d.fromDegrees(60)));
    branchRobotPositions.add(new Pose2d(12.826, 4.78, Rotation2d.fromDegrees(60)));
    branchRobotPositions.add(new Pose2d(13.329, 4.810, Rotation2d.fromDegrees(0)));
    branchRobotPositions.add(new Pose2d(13.617, 4.631, Rotation2d.fromDegrees(0)));
    branchRobotPositions.add(new Pose2d(13.894, 4.2, Rotation2d.fromDegrees(-60)));
    branchRobotPositions.add(new Pose2d(13.894, 3.875, Rotation2d.fromDegrees(-60)));
    branchRobotPositions.add(new Pose2d(13.629, 3.396, Rotation2d.fromDegrees(-120)));
    branchRobotPositions.add(new Pose2d(13.353, 3.264, Rotation2d.fromDegrees(-120)));
    branchRobotPositions.add(new Pose2d(12.814, 3.228, Rotation2d.fromDegrees(180)));
    branchRobotPositions.add(new Pose2d(12.502, 3.396, Rotation2d.fromDegrees(180)));
    branchRobotPositions.add(new Pose2d(12.294, 3.863, Rotation2d.fromDegrees(120)));
    branchRobotPositions.add(new Pose2d(12.294, 4.2, Rotation2d.fromDegrees(120)));
  }

  public Pose2d getNearestBranch() {
    return drivetrain.getState().Pose.nearest(branchRobotPositions);
  }

  public Rotation3d getRotationAngle() {
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    System.out.println(nearestPose.getRotation().getDegrees());
    switch ((int) nearestPose.getRotation().getDegrees()) {
      case 119:
        pitch = Units.degreesToRadians(30); // Front Middle
        break;
      case 59:
        pitch = Units.degreesToRadians(-30); // Front Left
        yaw = Units.degreesToRadians(120);
        break;
      case 0:
        pitch = Units.degreesToRadians(-30); // Back Right
        yaw = Units.degreesToRadians(60);
        break;
      case -59:
        pitch = Units.degreesToRadians(-30); // Back Middle
        break;
      case -119:
        pitch = Units.degreesToRadians(-30); // Back Left
        yaw = Units.degreesToRadians(-60);
        break;
      case 180:
        pitch = Units.degreesToRadians(-30); // Front Right
        yaw = Units.degreesToRadians(-120);
        break;
      default:
        break;
    }
    return new Rotation3d(roll, pitch, yaw);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {
    nearestPose = getNearestBranch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPose = new Pose3d(getNearestBranch());
    Rotation3d rotation = getRotationAngle();
    Pose3d coralToUse = new Pose3d(coralPose.getX(), coralPose.getY(), 0.77, rotation);
    canPlace = true;
    if (effectorSim.simulatedBeamBreak) {
      effector.reefCoral.add(coralToUse);
      effectorSim.simulatedBeamBreak = false;
      effector.updateArray();
      score();
    }
  }

  public void score() {
    if (DriverStation.isAutonomousEnabled()) {
      effector.simulatedRedScore = effector.simulatedRedScore += 4;
    } else if (DriverStation.isTeleopEnabled()) {
      effector.simulatedRedScore = effector.simulatedRedScore += 3;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return canPlace;
  }
  @Override
  public void end(boolean interrupted) {
    
  }
}
