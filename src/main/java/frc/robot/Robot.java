// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Random;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  Alert alert;

  public Robot() {
    m_robotContainer = new RobotContainer();
    alert = new Alert("Loading Motivational Speech Framework...", AlertType.kInfo);
    alert.set(true);
  }

  public void motivationalQuotes() {
    Random random = new Random();
    int choice = random.nextInt(30) + 1;
    switch (choice) {
      case 1:
        alert.setText("");
        break;
      case 2:
        alert.setText("");
        break;
      case 3:
        alert.setText("");
        break;
      case 4:
        alert.setText("");
        break;
      case 5:
        alert.setText("");
        break;
      case 6:
        alert.setText("");
        break;
      case 7:
        alert.setText("");
        break;
      case 8:
        alert.setText("");
        break;
      case 9:
        alert.setText("");
        break;
      case 10:
        alert.setText("");
        break;
      case 11:
        alert.setText("");
        break;
      case 12:
        alert.setText("");
        break;
      case 13:
        alert.setText("");
        break;
      case 14:
        alert.setText("");
        break;
      case 15:
        alert.setText("");
        break;
      case 16:
        alert.setText("");
        break;
      case 17:
        alert.setText("");
        break;
      case 18:
        alert.setText("");
        break;
      case 19:
        alert.setText("");
        break;
      case 20:
        alert.setText("");
        break;
      case 21:
        alert.setText("");
        break;
      case 22:
        alert.setText("");
        break;
      case 23:
        alert.setText("");
        break;
      case 24:
        alert.setText("");
        break;
      case 25:
        alert.setText("");
        break;
      case 26:
        alert.setText("");
        break;
      case 27:
        alert.setText("");
        break;
      case 28:
        alert.setText("");
        break;
      case 29:
        alert.setText("");
        break;
      case 30:
        alert.setText("");
        break;
        
        
    }
  }

  @Override
  public void robotInit() {
    for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
      m_robotContainer.limelight.teamAdd = 180;
    } else {
      m_robotContainer.limelight.teamAdd = 0;
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if(DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))) {
      m_robotContainer.limelight.teamAdd = 180;
    } else {
      m_robotContainer.limelight.teamAdd = 0;
    }
    
    motivationalQuotes();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}