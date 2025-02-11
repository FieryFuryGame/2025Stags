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
    int choice = random.nextInt(38) + 1;
    switch (choice) {
      case 1:
        alert.setText("You can do it!");
        break;
      case 2:
        alert.setText("Loading message failed: [click to view error]");
        break;
      case 3:
        alert.setText("Nearly there!");
        break;
      case 4:
        alert.setText("The other team isn't ahead by that much...");
        break;
      case 5:
        alert.setText("You're the best");
        break;
      case 6:
        alert.setText("Are you even reading these?");
        break;
      case 7:
        alert.setText("Nice!");
        break;
      case 8:
        alert.setText("It's not your fault.");
        break;
      case 9:
        alert.setText("Go STAGS!");
        break;
      case 10:
        alert.setText("GO GO GO!");
        break;
      case 11:
        alert.setText("Happy little accidents!");
        break;
      case 12:
        alert.setText("1262");
        break;
      case 13:
        alert.setText("youtube.com/watch?v=dQw4w9WgXcQ");
        break;
      case 14:
        alert.setText("Don't blame the coder, blame the code.");
        break;
      case 15:
        alert.setText("May contain if statements");
        break;
      case 16:
        alert.setText("Error: Math not mathing");
        break;
      case 17:
        alert.setText("NOT WATERPROOF!");
        break;
      case 18:
        alert.setText("-.-- --- ..- / .-- .- ... -. -.. / -.-- --- ..- .-. / - .. -- .");
        break;
      case 19:
        alert.setText("FIRST");
        break;
      case 20:
        alert.setText("Safety fifth!");
        break;
      case 21:
        alert.setText("2025");
        break;
      case 22:
        alert.setText("Do a flip!");
        break;
      case 23:
        alert.setText("We love Bubbles!");
        break;
      case 24:
        alert.setText("Let's go!");
        break;
      case 25:
        alert.setText("Amazing!");
        break;
      case 26:
        alert.setText("You learn from your mistakes.");
        break;
      case 27:
        alert.setText("Your dad loves you, Charley");
        break;
      case 28:
        alert.setText("Knock 'em out of the park!");
        break;
      case 29:
        alert.setText("QWERTY");
        break;
      case 30:
        alert.setText("One step at a time!");
        break;
      case 31:
        alert.setText("Do your best!");
        break;
      case 32:
        alert.setText("We support the rich!");
        break;
      case 33:
        alert.setText("Live, Laugh, Love!");
        break;
      case 34:
        alert.setText("Keep going!");
        break;
      case 35:
        alert.setText("Jonas was here!");
        break;
      case 36:
        alert.setText("Nothing is impossible!");
        break;
      case 37:
        alert.setText("We believe in you!");
        break;
      case 38:
        alert.setText("Don't give up!");
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
      if (llMeasurement != null && llMeasurement.tagCount > 0) {
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