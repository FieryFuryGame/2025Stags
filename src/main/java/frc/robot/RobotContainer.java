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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.EffectorPivot;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.SimulatePlacingCoralL2;
import frc.robot.commands.SimulatePlacingCoralL3;
import frc.robot.commands.SimulatePlacingCoralL4;
import frc.robot.commands.IntakeAlgae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.PhotonSim;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 1.6 rotations per second max angular velocity

    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    Field2d field2d = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final EndEffector effector = new EndEffector();
    public final Climber climber = new Climber();
    public final PhotonSim photonSim = new PhotonSim(drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Elevator Commands
        NamedCommands.registerCommand("L1", Commands.runOnce(() -> elevator.setLevelOne()));
        NamedCommands.registerCommand("L2", Commands.runOnce(() -> elevator.setLevelTwo()));
        NamedCommands.registerCommand("L3", Commands.runOnce(() -> elevator.setLevelThree()));
        NamedCommands.registerCommand("L4", new SimulatePlacingCoralL4(drivetrain, effector));
        NamedCommands.registerCommand("waitForL1", new WaitUntilCommand(elevator.isL4));
        NamedCommands.registerCommand("waitForL4", new WaitUntilCommand(elevator.isL4));

        // Effector Commands
        NamedCommands.registerCommand("dispenseCoral", effector.setWheelVoltageCommand(-12));
        NamedCommands.registerCommand("waitForCoral", new WaitUntilCommand(effector.checkBeam));
        NamedCommands.registerCommand("loadCoral", effector.setWheelVoltageCommand(-7).andThen(effector.setConveyorVoltageCommand(-6)));
        NamedCommands.registerCommand("stopEffector", effector.setWheelVoltageCommand(0).andThen(effector.setConveyorVoltageCommand(0.0)));
        NamedCommands.registerCommand("pivotEffector", Commands.runOnce(() -> new EffectorPivot(effector).execute()));

        // Swerve Commands
        NamedCommands.registerCommand("zeroGyro", Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto Selection
        autoChooser = AutoBuilder.buildAutoChooser("Top 2 L4");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drive and steer stuff
            drivetrain.applyRequest(() ->
                fieldDrive.withVelocityX(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Point wheels towards center
        Constants.OperatorConstants.driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Pathfinding control
        Constants.OperatorConstants.driverController.leftBumper().onTrue(Commands.runOnce(() -> photonSim.pathfindWithPath("Left").schedule()).unless(() -> photonSim.tid <= 0));
        Constants.OperatorConstants.driverController.rightBumper().onTrue(Commands.runOnce(() -> photonSim.pathfindWithPath("Right").schedule()).unless(() -> photonSim.tid <= 0));
        Constants.OperatorConstants.driverController.y().onTrue(Commands.runOnce(() -> photonSim.pathfindWithPath("Center").schedule()).unless(() -> photonSim.tid <= 0));
        
        // Miscellaneous
        Constants.OperatorConstants.driverController.x().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        Constants.OperatorConstants.driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Elevator Manual Control
        Constants.OperatorConstants.operatorController.leftTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(2)))
            .onFalse(elevator.setVoltage(0));
        Constants.OperatorConstants.operatorController.rightTrigger()
            .onTrue(elevator.stopElevator().andThen(elevator.setVoltage(-2)))
            .onFalse(elevator.setVoltage(0));
        
        // Elevator Automatic Control    
        Constants.OperatorConstants.driverController.povUp().onTrue(new SimulatePlacingCoralL3(drivetrain, effector));
        Constants.OperatorConstants.driverController.povLeft().onTrue(new SimulatePlacingCoralL2(drivetrain, effector));
        Constants.OperatorConstants.driverController.povRight().onTrue(new SimulatePlacingCoralL4(drivetrain, effector));

        // Effector Controls
        Constants.OperatorConstants.operatorController.a().whileTrue(new LoadCoral(effector));
        Constants.OperatorConstants.operatorController.b().whileTrue(new EjectCoral(effector));
        Constants.OperatorConstants.operatorController.x().whileTrue(new IntakeAlgae(effector));
        Constants.OperatorConstants.operatorController.y().onTrue(Commands.runOnce(() -> new EffectorPivot(effector).execute()));
        
        // Fancy logging stuff that fills all the storage on the RIO
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}