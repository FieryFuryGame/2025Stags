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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.EffectorPivot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffector.WheelState;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 1.6 rotations per second max angular velocity

    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    CommandXboxController testController = new CommandXboxController(2);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    Field2d field2d = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final EndEffector effector = new EndEffector();
    public final Limelight limelight = new Limelight("limelight", drivetrain);
    // public final Climber climber = new Climber();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Elevator Commands
        NamedCommands.registerCommand("L1", elevator.setState(ElevatorState.GROUND));
        NamedCommands.registerCommand("L2", elevator.setState(ElevatorState.L2));
        NamedCommands.registerCommand("L3", elevator.setState(ElevatorState.L3));
        NamedCommands.registerCommand("L4", elevator.setState(ElevatorState.L4));
        NamedCommands.registerCommand("waitForL1", new WaitUntilCommand(elevator.isL4));
        NamedCommands.registerCommand("waitForL4", new WaitUntilCommand(elevator.isL4));

        // Effector Commands
        NamedCommands.registerCommand("dispenseCoral", effector.setWheelState(WheelState.Eject).asProxy());
        NamedCommands.registerCommand("waitForCoral", new WaitUntilCommand(effector.checkBeam));
        NamedCommands.registerCommand("loadCoral", effector.setWheelState(WheelState.IntakeCoral).asProxy());
        NamedCommands.registerCommand("stopEffector", effector.setWheelState(WheelState.Idle));
        NamedCommands.registerCommand("pivotEffector", Commands.runOnce(() -> new EffectorPivot(effector).asProxy().execute()));

        // Swerve Commands
        NamedCommands.registerCommand("zeroGyro", Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto Selection
        autoChooser = AutoBuilder.buildAutoChooser("Backup");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drive and steer stuff
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Point wheels towards center
        Constants.OperatorConstants.driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Pathfinding control
        Constants.OperatorConstants.driverController.leftBumper().onTrue(new AlignToReef(drivetrain, "Left"));
        Constants.OperatorConstants.driverController.rightBumper().onTrue(new AlignToReef(drivetrain, "Right"));
        Constants.OperatorConstants.driverController.y().onTrue(Commands.runOnce(() -> limelight.pathfindWithPath("Center").schedule()).unless(() -> limelight.tid.getDouble(0.0) <= 0));
        
        // Miscellaneous
        Constants.OperatorConstants.driverController.x().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        Constants.OperatorConstants.driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /*
        Constants.OperatorConstants.driverController.povDown().onTrue(climber.moveToSetpointCommand(0.0)); // Initial Position
        Constants.OperatorConstants.driverController.povLeft().onTrue(climber.moveToSetpointCommand(0.0)); // Preparing Position
        Constants.OperatorConstants.driverController.povRight().onTrue(climber.moveToSetpointCommand(0.0)); // Raising Position

        Constants.OperatorConstants.driverController.leftTrigger()
            .onTrue(climber.setVoltageCommand(12))
            .onFalse(climber.setVoltageCommand(0));
        Constants.OperatorConstants.driverController.rightTrigger()
            .onTrue(climber.setVoltageCommand(-12))
            .onFalse(climber.setVoltageCommand(0));
        */
        // Elevator Manual Control
        Constants.OperatorConstants.operatorController.leftTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(2)).andThen(elevator.setState(ElevatorState.MANUAL)))
            .onFalse(elevator.setVoltage(0));
        Constants.OperatorConstants.operatorController.rightTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(-2)).andThen(elevator.setState(ElevatorState.MANUAL)))
            .onFalse(elevator.setVoltage(0));
        
        // Elevator Automatic Control    
        Constants.OperatorConstants.operatorController.povUp().onTrue(elevator.setState(ElevatorState.L3));
        Constants.OperatorConstants.operatorController.povLeft().onTrue(elevator.setState(ElevatorState.L2));
        Constants.OperatorConstants.operatorController.povRight().onTrue(elevator.setState(ElevatorState.L4));
        Constants.OperatorConstants.operatorController.povDown().onTrue(elevator.setState(ElevatorState.GROUND));

        // Effector Controls
        Constants.OperatorConstants.operatorController.a().whileTrue(effector.setWheelState(WheelState.IntakeCoral).onlyIf(() -> !effector.checkBeam.getAsBoolean())).onFalse(effector.setWheelState(WheelState.Idle));
        Constants.OperatorConstants.operatorController.b().whileTrue(effector.setWheelState(WheelState.Eject)).onFalse(effector.setWheelState(WheelState.Idle));
        Constants.OperatorConstants.operatorController.x().whileTrue(effector.setWheelState(WheelState.IntakeAlgae)).onFalse(effector.setWheelState(WheelState.Idle));
        Constants.OperatorConstants.operatorController.y().onTrue(Commands.runOnce(() -> new EffectorPivot(effector).execute()));
        /*
        testController.leftTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(12)))
            .onFalse(elevator.setVoltage(0));
        testController.rightTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(-12)))
            .onFalse(elevator.setVoltage(0));
        */

        new Trigger(effector.checkBeam)
            .and(() -> effector.wheelState == WheelState.IntakeCoral)
            .onTrue(new WaitCommand(0.2)
            .andThen(effector.setWheelState(WheelState.Idle)));
        
        // Fancy logging stuff that fills all the storage on the RIO
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}