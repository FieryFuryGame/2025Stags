// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final int ElevatorMotorID = 10;
    // PID for motor B
    public static final double kS = 0.25, kV = 0.12, kA = 0.01, kP = 4.8, kI = 0, kD = 0.1, kG = 0.2;
    // Motion Magic Config B
    public static final double mmCruiseVelocity = 200, mmAccel = 400, mmJerk = 1600;
  }

  public static class EndEffectorConstants {
    public static final int EffectorWheelsID = 11;

    public static final int EffectorPivotID = 12;
    // PID for motor B
    public static final double pivotkS = 0.25, pivotkV = 0.12, pivotkA = 0.01, pivotkP = 4.8, pivotkI = 0, pivotkD = 0.1;
    // Motion Magic Config B
    public static final double pivotmmCruiseVelocity = 80, pivotmmAccel = 160, pivotmmJerk = 1600;
  }

  public static class FloorIntakeConstants {
    public static final int leftPivotMotorID = 13;
    public static final int leftWheelsMotorID = 14;
    public static final int rightPivotMotorID = 15;
    public static final int rightWheelsMotorID = 16;

    public static final double leftkS = 0.25, leftkV = 0.12, leftkA = 0.01, leftkP = 4.8, leftkI = 0, leftkD = 0.1;
    public static final double rightkS = 0.25, rightkV = 0.12, rightkA = 0.01, rightkP = 4.8, rightkI = 0, rightkD = 0.1;

    public static final double leftmmCruiseVelocity = 80, leftmmAccel = 160, leftmmJerk = 1600;
    public static final double rightmmCruiseVelocity = 80, rightmmAccel = 160, rightmmJerk = 1600;
  }

  public static class DeepCageConstants {
    public static final int deepCageMotorID = 17;
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

  public static class AlignmentConstants {
    public static final Pose2d A_BLUE = new Pose2d(3.1, 4.19, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d A_RED = new Pose2d(14.381, 3.862, new Rotation2d(Math.toRadians(0)));

    public static final Pose2d B_BLUE = new Pose2d(3.1, 3.83, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d B_RED = new Pose2d(14.386, 4.163, new Rotation2d(Math.toRadians(0)));

    public static final Pose2d C_BLUE = new Pose2d(3.662, 2.963, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d C_RED = new Pose2d(13.871, 5.079, new Rotation2d(Math.toRadians(60)));

    public static final Pose2d D_BLUE = new Pose2d(3.9, 2.818, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d D_RED = new Pose2d(13.579, 5.231, new Rotation2d(Math.toRadians(60)));

    public static final Pose2d E_BLUE = new Pose2d(5.03, 2.828, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d E_RED = new Pose2d(12.531, 5.229, new Rotation2d(Math.toRadians(120)));

    public static final Pose2d F_BLUE = new Pose2d(5.320, 2.963, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d F_RED = new Pose2d(12.261, 5.076, new Rotation2d(Math.toRadians(120)));

    public static final Pose2d G_BLUE = new Pose2d(5.807, 3.833, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d G_RED = new Pose2d(11.750, 4.183, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d H_BLUE = new Pose2d(5.807, 4.175, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d H_RED = new Pose2d(11.750, 3.858, new Rotation2d(Math.toRadians(180)));

    public static final Pose2d I_BLUE = new Pose2d(5.289, 5.077, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d I_RED = new Pose2d(12.258, 2.971, new Rotation2d(Math.toRadians(240)));

    public static final Pose2d J_BLUE = new Pose2d(5.019, 5.232, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d J_RED = new Pose2d(12.544, 2.802, new Rotation2d(Math.toRadians(240)));

    public static final Pose2d K_BLUE = new Pose2d(3.692, 5.062, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d K_RED = new Pose2d(13.564, 2.804, new Rotation2d(Math.toRadians(300)));

    public static final Pose2d L_BLUE = new Pose2d(3.074, 5.056, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d L_RED = new Pose2d(13.564, 2.804, new Rotation2d(Math.toRadians(300)));
  }
}
