// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSimExtra;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSimExtra;
import frc.robot.subsystems.ExtraDriver;

// import frc.robot.subsystems.CANLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class DecideWhereToPlaceCoralExtra extends Command {
  ElevatorSimExtra elevatorSim;
  ExtraDriver drivetrain;
  EndEffector effector;
  EndEffectorSimExtra effectorSim;
  int branch = 0;
  boolean isFinished = false;

  /** Creates a new LaunchNote. */
  public DecideWhereToPlaceCoralExtra(ElevatorSimExtra elevator, ExtraDriver drivetrain, EndEffector effector, EndEffectorSimExtra effectorSim) {
    // save the launcher system internally
    elevatorSim = elevator;
    this.drivetrain = drivetrain;
    this.effector = effector;
    this.effectorSim = effectorSim;
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSim.percentageUp > 20) {
        if (elevatorSim.percentageUp > 35 && elevatorSim.percentageUp < 45) {
            branch = 2;
        }
        if (elevatorSim.percentageUp > 59 && elevatorSim.percentageUp < 69) {
            branch = 3;
        }
        if (elevatorSim.percentageUp > 90) {
            branch = 4;
        }

        if (branch == 2) {
            new SimulatePlacingCoralL2Extra(drivetrain, effector, effectorSim).schedule();
        } else if (branch == 3) {
            new SimulatePlacingCoralL3Extra(drivetrain, effector, effectorSim).schedule();
        } else if (branch == 4) {
            new SimulatePlacingCoralL4Extra(drivetrain, effector, effectorSim).schedule();
        }
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

  }
}
