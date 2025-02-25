// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Random;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.FollowPathCommand;

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
    alert = new Alert("Loading Motivation-ish Framework...", AlertType.kInfo);
    alert.set(true);
  }

  public void motivationalQuotes() { // Joke we are doing for the drivers during testing.
    Random random = new Random();
    int choice = random.nextInt(84) + 1;
    switch (choice) {
      // A missing semi-colon
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
        alert.setText("Nice, but less important");
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
      case 39:
        alert.setText("FEAR THE DEER!!!");
        break;
      case 40:
        alert.setText("This is a message supplying support for you to win this match by gaining more points than another team during this match. And therefore, you are motivated. Hooray.");
        break;
      case 41:
        alert.setText("You will win, and then there will be cake.");
        break;
      case 42:
        alert.setText("Keep calm and place coral");
        break;
      case 43:
        alert.setText("We have been trying to contact you about your car's extended warranty.");
        break;
      case 44:
        alert.setText("Charley enabled the robot.");
        break;
      case 45:
        alert.setText("System.out.println(\"You're doing great!\")");
        break;
      case 46:
        alert.setText("01001101 01101111 01110100 01101001 01110110 01100001 01110100 01101001 01101111 01101110 00101110");
        break;
      case 47:
        alert.setText("Let's go STAGS! Let's go!");
        break;
      case 48:
        alert.setText("Made with 0% recycled minerals");
        break;
      case 49:
        alert.setText("3. 2. 1. Reefscape!");
        break;
      case 50:
        alert.setText("(Don't tell Charley)");
        break;
      case 51:
        alert.setText("Safety glasses!");
        break;
      case 52:
        alert.setText("alert.setText(\"\");");
        break;
      case 53:
        alert.setText("I used the code to delete the code.");
        break;
      case 54:
        alert.setText("Charley looked at the text in confusion, as the programmers laughed about their silly joke. This is the joke.");
        break;
      case 55:
        alert.setText("!yelrahC llet t'noD");
        break;
      case 56:
        alert.setText("[Robot] wants to know your location");
        break;
      case 57:
        alert.setText("The fitness gram pacer test is a multi-stage aerobics test that gets progressively harder each stage.");
        break;
      case 58:
        alert.setText("What are you doing in my swamp!");
        break;
      case 100:
        alert.setText("This message exists, but will never be seen. Isn't that weird?");
        break;
      case 59:
        alert.setText("Either you run the robot, or the robot runs you.");
        break;
      case 60:
        alert.setText("99% of drivers lose a match before they place coral. Not you.");
        break;
      case 61:
        alert.setText("minecraft/assets/textures/model/lightly_waxed_weathered_cut_copper_stairs.json");
        break;
      case 62:
        alert.setText("The code died. Sorry.");
        break;
      case 63:
        alert.setText("Roses are red, violets are blue. There is a semi-colon missing, on line 42.");
        break;
      case 64:
        alert.setText("If aliens invade earth, show them this message. Hi.");
        break;
      case 65:
        alert.setText("Totally Legal™");
        break;
      case 66:
        alert.setText("When every robot is going left, go right.");
        break;
      case 67:
        alert.setText("Nice, but more important");
        break;
      case 68:
        alert.setText("Programming 2: Electric Boogaloo");
        break;
      case 69:
        alert.setText("The attack was super effective!");
        break;
      case 70:
        alert.setText("Finish him");
        break;
      case 71:
        alert.setText("What would FIRST think of this?");
        break;
      case 72:
        alert.setText("Someone should write a research paper about this text box.");
        break;
      case 73:
        alert.setText("Do you like chemistry jokes? Actually, I was going to tell one about Sodium and Hydrogen. But NaH.");
        break;
      case 74:
        alert.setText("Yes, of course there's a point to this!");
        break;
      case 75:
        alert.setText("Also try FTC!");
        break;
      case 76:
        alert.setText("Long live the king!");
        break;
      case 77:
        alert.setText("Jonas joined the game.");
        break;
      case 78:
        alert.setText("Was Jonas here?");
        break;
      case 79:
        alert.setText("May contain switch statements");
        break;
      case 80:
        alert.setText("Collapse the children! (This is fine in context lol)");
        break;
      case 81:
        alert.setText("This message was not written on April 78th, -20068 CE, at 25:76 RM");
        break;
      case 82:
        alert.setText("Ctrl + S");
        break;
      case 83:
        alert.setText("Does Hercules wear whitey tighties?");
        break;
      case 84:
        alert.setText("So, does it work?");
        break;
    }
  }

  @Override
  public void robotInit() {
    for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
    }
    FollowPathCommand.warmupCommand().schedule();
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