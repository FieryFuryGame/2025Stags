// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
  }

  public static class ElevatorConstants {
    public static final int ElevatorMotorAID = 9;
    // PID for motor A
    public static final double akS = 0.25, akV = 0.12, akA = 0.01, akP = 4.8, akI = 0, akD = 0.1;
    // Motion Magic Config A
    public static final double ammCruiseVelocity = 80, ammAccel = 160, ammJerk = 1600;

    public static final int ElevatorMotorBID = 10;
    // PID for motor B
    public static final double bkS = 0.25, bkV = 0.12, bkA = 0.01, bkP = 4.8, bkI = 0, bkD = 0.1;
    // Motion Magic Config B
    public static final double bmmCruiseVelocity = 80, bmmAccel = 160, bmmJerk = 1600;
  }

  public static class TagConstants {
    public static final int Tag9Angle = 300;
    public static final int Tag8Angle = 240;
    public static final int Tag7Angle = 180;
    public static final int Tag6Angle = 120;
    public static final int Tag11Angle = 60;
    public static final int Tag10Angle = 0;

    public static final int Tag19Angle = 300;
    public static final int Tag20Angle = 240;
    public static final int Tag21Angle = 180;
    public static final int Tag22Angle = 120;
    public static final int Tag17Angle = 60;
    public static final int Tag18Angle = 0;

  }
}
