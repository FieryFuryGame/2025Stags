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

    public static final int ElevatorMotorBID = 9;
    public static final int ElevatorMotorAID = 10;
    // PID for motor B
    public static final double kS = 0.25, kV = 0.12, kA = 0.01, kP = 4.8, kI = 0, kD = 0.332, kG = 0.3;
    // Motion Magic Config B
    public static final double mmCruiseVelocity = 93, mmAccel = 450, mmJerk = 1600;
  }

  public static class EndEffectorConstants {
    public static final int EffectorWheelsID = 11;
    public static final int EffectorPivotID = 12;
    public static final int ConveyorID = 13;
    // PID for Pivot
    public static final double pivotkS = 0.25, pivotkV = 0.12, pivotkA = 0.01, pivotkP = 4.8, pivotkI = 0, pivotkD = 0.1;
    // Pivot Motion Magic Config
    public static final double pivotmmCruiseVelocity = 80, pivotmmAccel = 160, pivotmmJerk = 1600;
  }

  public static class ClimberConstants {
    public static final int ClimberMotorA = 14;
    public static final int ClimberMotorB = 15;

    public static final double kS = 0.25, kV = 0.12, kA = 0.01, kP = 4.8, kI = 0, kD = 0.1, kG = 0.3;
    public static final double mmCruiseVelocity = 80, mmAccel = 160, mmJerk = 1600;
  }
}
