// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;

public class FloorIntakeRightToggle extends Command {
  FloorIntake m_intake;

  /** Creates a new LaunchNote. */
  public FloorIntakeRightToggle(FloorIntake intake) {
    // save the launcher system internally
    m_intake = intake;

    // indicate that this command requires the launcher system
    addRequirements(intake);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.rightDown) {
        m_intake.RightMM(0);
        m_intake.rightDown = false;
    } else {
        m_intake.RightMM(0);
        m_intake.rightDown = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return false;
  }
  @Override
  public void end(boolean interrupted) {
    
  }
}
