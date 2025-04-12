// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffector;

// import frc.robot.subsystems.CANLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class SimulateCoralIntake extends Command {
  CommandSwerveDrivetrain drivetrain;
  EndEffector effector;

  List<Pose2d> stationPoses = new ArrayList<Pose2d>();

  PathConstraints constraints = new PathConstraints(
        6.0, 5.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

  boolean isFinished = false;

  public SimulateCoralIntake(CommandSwerveDrivetrain drivetrain, EndEffector effector) {
    this.drivetrain = drivetrain;
    this.effector = effector;

    stationPoses.add(new Pose2d(1.199, 7.052, Rotation2d.fromDegrees(-54.46)));
    stationPoses.add(new Pose2d(1.199, 0.998, Rotation2d.fromDegrees(54.46)));
  }

  public Pose2d getNearest() {
    return drivetrain.getState().Pose.nearest(stationPoses);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivePose = drivetrain.getState().Pose;
    Pose2d nearestPose = getNearest();
    double distance = Math.sqrt(Math.pow((nearestPose.getX() - drivePose.getX()), 2) + Math.pow((nearestPose.getY() - nearestPose.getY()), 2));
    System.out.println(distance);
    if (distance < 0.5) {
      effector.simulatedBeamBreak = true;
    }
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return isFinished;
  }
  
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    
  }
}
