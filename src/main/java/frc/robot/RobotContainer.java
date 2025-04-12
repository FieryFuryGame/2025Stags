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
import frc.robot.commands.DecideWhereToPlaceCoral;
import frc.robot.commands.EffectorPivot;
import frc.robot.commands.SimulateCoralIntake;
import frc.robot.commands.SimulatePlacingCoralL4;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSim;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSim;
import frc.robot.subsystems.PhotonSim;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 1.6 rotations per second max angular velocity

    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final Telemetry logger = new Telemetry(MaxSpeed);
    Field2d field2d = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final EndEffector effector = new EndEffector();
    public final Climber climber = new Climber();
    public final PhotonSim photonSim = new PhotonSim(drivetrain);
    public final ElevatorSim elevatorSim = new ElevatorSim();
    public final EndEffectorSim effectorSim = new EndEffectorSim(effector, elevatorSim, drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Elevator Commands
        NamedCommands.registerCommand("L1", elevatorSim.simulateSetpoint(0));
        NamedCommands.registerCommand("L2", elevatorSim.simulateSetpoint(40));
        NamedCommands.registerCommand("L3", elevatorSim.simulateSetpoint(64));
        NamedCommands.registerCommand("L4", elevatorSim.simulateSetpoint(100));
        NamedCommands.registerCommand("waitForL1", new WaitUntilCommand(elevatorSim.isL1));
        NamedCommands.registerCommand("waitForL4", new WaitUntilCommand(elevatorSim.isL4));

        // Effector Commands
        NamedCommands.registerCommand("dispenseCoral", new DecideWhereToPlaceCoral(elevatorSim, drivetrain, effector));
        NamedCommands.registerCommand("dispenseL4Coral", new SimulatePlacingCoralL4(drivetrain, effector));
        NamedCommands.registerCommand("waitForCoral", new WaitUntilCommand(effector.checkBeam));
        NamedCommands.registerCommand("loadCoral", new SimulateCoralIntake(drivetrain, effector));
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
        Constants.OperatorConstants.driverController.a().onTrue(new SimulateCoralIntake(drivetrain, effector));
        Constants.OperatorConstants.driverController.b().onTrue(new DecideWhereToPlaceCoral(elevatorSim, drivetrain, effector));

        // Pathfinding control
        Constants.OperatorConstants.driverController.leftBumper().onTrue(Commands.runOnce(() -> photonSim.pathfindWithPath("Left").schedule()).unless(() -> photonSim.tid <= 0));
        Constants.OperatorConstants.driverController.rightBumper().onTrue(Commands.runOnce(() -> photonSim.pathfindWithPath("Right").schedule()).unless(() -> photonSim.tid <= 0));
        Constants.OperatorConstants.driverController.y().onTrue(Commands.runOnce(() -> effector.clearArray()));
        
        // Miscellaneous
        Constants.OperatorConstants.driverController.x().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        
        // Elevator Automatic Control    
        Constants.OperatorConstants.driverController.povUp().onTrue(elevatorSim.simulateSetpoint(64));
        Constants.OperatorConstants.driverController.povLeft().onTrue(elevatorSim.simulateSetpoint(40));
        Constants.OperatorConstants.driverController.povRight().onTrue(elevatorSim.simulateSetpoint(100));
        Constants.OperatorConstants.driverController.povDown().onTrue(elevatorSim.simulateSetpoint(0));

        Constants.OperatorConstants.driverController.leftTrigger().onTrue(Commands.runOnce(() -> elevatorSim.manualControl = true));
        Constants.OperatorConstants.driverController.rightTrigger().onTrue(Commands.runOnce(() -> elevatorSim.manualControl = true));
        
        // Fancy logging stuff that fills all the storage on the RIO
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}