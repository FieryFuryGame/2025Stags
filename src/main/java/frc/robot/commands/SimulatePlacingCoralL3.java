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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffector;

// import frc.robot.subsystems.CANLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class SimulatePlacingCoralL3 extends Command {
  CommandSwerveDrivetrain drivetrain;
  EndEffector effector;

  Pose3d coralPose = new Pose3d();
  List<Pose2d> branchRobotPositions = new ArrayList<Pose2d>();
  Pose2d nearestPose = new Pose2d();
  boolean canPlace = false;
  int branchID = 3;

  public SimulatePlacingCoralL3(CommandSwerveDrivetrain drivetrain, EndEffector effector) {
    this.drivetrain = drivetrain;
    this.effector = effector;

    branchRobotPositions.add(new Pose2d(3.7, 3.863, Rotation2d.fromDegrees(120)));
    branchRobotPositions.add(new Pose2d(3.7, 4.2, Rotation2d.fromDegrees(120)));

    this.addRequirements(effector);
  }

  public Pose2d getNearestBranch() {
    return drivetrain.getState().Pose.nearest(branchRobotPositions);
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
    Rotation3d rotation = new Rotation3d(0, Units.degreesToRadians(25), 0);
    double[] coralPoseArray = {coralPose.getX(), coralPose.getY(), 1.2, rotation.getAngle(), rotation.getX(), rotation.getY(), rotation.getZ()};
    SmartDashboard.putNumberArray(Integer.toString(branchID), coralPoseArray);
    canPlace = true;
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
