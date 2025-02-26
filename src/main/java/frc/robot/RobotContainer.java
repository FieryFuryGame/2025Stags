// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DeepCage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    // public final EndEffector m_effector = new EndEffector();
    // public final FloorIntake m_floorIntake = new FloorIntake();
    // public final DeepCage m_deepCage = new DeepCage();
    public final Limelight limelight = new Limelight("limelight", drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        /*
        NamedCommands.registerCommand("L1", Commands.runOnce(() -> m_elevator.setLevelOne()));
        NamedCommands.registerCommand("L2", Commands.runOnce(() -> m_elevator.setLevelTwo()));
        NamedCommands.registerCommand("L3", Commands.runOnce(() -> m_elevator.setLevelThree()));
        NamedCommands.registerCommand("L4", Commands.runOnce(() -> m_elevator.setLevelFour()));
        
        NamedCommands.registerCommand("dispenseCoral", m_effector.runEffector());
        */

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getRightX(), 0.08) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        Constants.OperatorConstants.driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.OperatorConstants.driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Constants.OperatorConstants.driverController.getLeftY(), -Constants.OperatorConstants.driverController.getLeftX()))
        ));

        Constants.OperatorConstants.driverController.leftBumper().onTrue(Commands.runOnce(() -> limelight.pathfindWithPath("Left").schedule()));//.unless(() -> limelight.tid.getDouble(0.0) <= 0));
        Constants.OperatorConstants.driverController.rightBumper().onTrue(Commands.runOnce(() -> limelight.pathfindWithPath("Right").schedule()));//.unless(() -> limelight.tid.getDouble(0.0) <= 0));
        Constants.OperatorConstants.driverController.y().onTrue(Commands.runOnce(() -> limelight.pathfindWithPath("Center").schedule()));//.unless(() -> limelight.tid.getDouble(0.0) <= 0));
        Constants.OperatorConstants.driverController.x().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Constants.OperatorConstants.driverController.back().and(Constants.OperatorConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.OperatorConstants.driverController.back().and(Constants.OperatorConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.OperatorConstants.driverController.start().and(Constants.OperatorConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.OperatorConstants.driverController.start().and(Constants.OperatorConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Constants.OperatorConstants.driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        Constants.OperatorConstants.driverController.povUp().onTrue(Commands.runOnce(() -> limelight.override += 1));
        Constants.OperatorConstants.driverController.povDown().onTrue(Commands.runOnce(() -> limelight.override -= 1));
        
        Constants.OperatorConstants.operatorController.leftTrigger()
            .whileTrue(m_elevator.setVoltage(6))
            .onFalse(m_elevator.setVoltage(0));
        Constants.OperatorConstants.operatorController.rightTrigger()
            .whileTrue(m_elevator.setVoltage(-6))
            .onFalse(m_elevator.setVoltage(0));
        /*
        Constants.OperatorConstants.operatorController.povDown().onTrue(Commands.runOnce(() -> m_elevator.setLevelThree()));
        Constants.OperatorConstants.operatorController.povRight().onTrue(Commands.runOnce(() -> m_elevator.setLevelTwo()));
        Constants.OperatorConstants.operatorController.povLeft().onTrue(Commands.runOnce(() -> m_elevator.setLevelFour()));
        Constants.OperatorConstants.operatorController.povUp().onTrue(Commands.runOnce(() -> m_elevator.setLevelOne()));
        */

        // Constants.OperatorConstants.operatorController.a().onTrue(m_effector.runEffector());

        // Constants.OperatorConstants.operatorController.leftBumper().onTrue(m_floorIntake.leftToggle());
        // Constants.OperatorConstants.operatorController.rightBumper().onTrue(m_floorIntake.rightToggle());
        
        /*
        Constants.OperatorConstants.operatorController.x().onTrue(m_floorIntake.intake().until(() -> m_floorIntake.floorLeftLoaded() || m_floorIntake.floorRightLoaded()).andThen(
            m_floorIntake.powerLeftIntake(0.0).alongWith(m_floorIntake.powerRightIntake(0.0))
        ));

        Constants.OperatorConstants.operatorController.y().onTrue(m_floorIntake.eject().andThen(new WaitCommand(0.5))
            .andThen(m_floorIntake.powerLeftIntake(0.0).alongWith(m_floorIntake.powerRightIntake(0.0))));
         */

        // Constants.OperatorConstants.driverController.leftTrigger().whileTrue(m_deepCage.move(0.0)).onFalse(m_deepCage.move(0.0));
        // Constants.OperatorConstants.driverController.rightTrigger().whileTrue(m_deepCage.move(0.0)).onFalse(m_deepCage.move(0.0));

        // Constants.OperatorConstants.driverController.leftTrigger().whileTrue(m_deepCage.moveWithLimitUp(0.0)).onFalse(m_deepCage.move(0.0));
        // Constants.OperatorConstants.driverController.rightTrigger().whileTrue(m_deepCage.moveWithLimitDown(0.0)).onFalse(m_deepCage.move(0.0));
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}