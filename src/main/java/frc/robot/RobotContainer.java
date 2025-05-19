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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.AlignToAlgae;
import frc.robot.commands.AlignToBranch;
import frc.robot.commands.DecideWhereToPlaceCoral;
import frc.robot.commands.EffectorPivot;
import frc.robot.commands.Eject;
import frc.robot.commands.EjectExtra;
import frc.robot.commands.SimulateAlgaeIntake;
import frc.robot.commands.SimulateAlgaeIntakeExtra;
import frc.robot.commands.SimulateCoralIntake;
import frc.robot.commands.SimulateCoralIntakeExtra;
import frc.robot.commands.SimulatePlacingCoralL4;
import frc.robot.generated.ExtraDriverConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BargeController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSim;
import frc.robot.subsystems.ElevatorSimExtra;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSim;
import frc.robot.subsystems.EndEffectorSimExtra;
import frc.robot.subsystems.ExtraDriver;
import frc.robot.subsystems.ElevatorSim.ElevatorState;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 1.6 rotations per second max angular velocity

    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric fieldDrive2 = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final Telemetry logger = new Telemetry(MaxSpeed);
    Field2d field2d = new Field2d();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final EndEffector effector = new EndEffector();
    public final BargeController bargeController = new BargeController(effector);
    public final Climber climber = new Climber();
    //public final PhotonSim photonSim = new PhotonSim(drivetrain);
    public final ElevatorSim elevatorSim = new ElevatorSim();
    public final EndEffectorSim effectorSim = new EndEffectorSim(effector, elevatorSim, drivetrain);
    public final ExtraDriver extraDriver = ExtraDriverConstants.createDrivetrain();
    public final ElevatorSimExtra elevatorSimExtra = new ElevatorSimExtra();
    public final EndEffectorSimExtra effectorSimExtra = new EndEffectorSimExtra(elevatorSimExtra, extraDriver);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Elevator Commands
        NamedCommands.registerCommand("L1", elevatorSim.setState(ElevatorState.GROUND));
        NamedCommands.registerCommand("L2", elevatorSim.setState(ElevatorState.L2));
        NamedCommands.registerCommand("L3", elevatorSim.setState(ElevatorState.L3));
        NamedCommands.registerCommand("L4", elevatorSim.setState(ElevatorState.L4));
        NamedCommands.registerCommand("waitForL1", new WaitUntilCommand(elevatorSim.isL1));
        NamedCommands.registerCommand("waitForL4", new WaitUntilCommand(elevatorSim.isL4));

        // Effector Commands
        NamedCommands.registerCommand("dispenseCoral", new DecideWhereToPlaceCoral(elevatorSim, drivetrain, effector));
        NamedCommands.registerCommand("dispenseL4Coral", new SimulatePlacingCoralL4(drivetrain, effector));
        NamedCommands.registerCommand("waitForCoral", new WaitUntilCommand(effector.checkBeam));
        NamedCommands.registerCommand("loadCoral", new SimulateCoralIntake(drivetrain, effector, effectorSim));
        NamedCommands.registerCommand("stopEffector", effector.setWheelVoltageCommand(0).andThen(effector.setConveyorVoltageCommand(0.0)));
        NamedCommands.registerCommand("pivotEffector", Commands.runOnce(() -> new EffectorPivot(effector).execute()));
        NamedCommands.registerCommand("leaveStartScore", Commands.runOnce(() -> effector.simulatedBlueScore = effector.simulatedBlueScore += 3));

        // Swerve Commands
        NamedCommands.registerCommand("zeroGyro", Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        // Auto Selection
        autoChooser = AutoBuilder.buildAutoChooser("Top 2 L4");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("Reset Simulation", Commands.runOnce(() -> {
            effector.reset();
            effectorSim.hasAlgae = false;
            effectorSimExtra.hasAlgae = false;
            effectorSimExtra.simulatedBeamBreak = true;
            drivetrain.resetPose(new Pose2d(7.588, 6.692, Rotation2d.k180deg));
            extraDriver.resetPose(new Pose2d(9.962, 1.322, Rotation2d.kZero));
        }).ignoringDisable(true));

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

        extraDriver.setDefaultCommand(
            // Drive and steer stuff
            extraDriver.applyRequest(() ->
                fieldDrive2.withVelocityX(-MathUtil.applyDeadband(Constants.OperatorConstants.operatorController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(Constants.OperatorConstants.operatorController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(Constants.OperatorConstants.operatorController.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Point wheels towards center
        Constants.OperatorConstants.driverController.a().onTrue(new SimulateCoralIntake(drivetrain, effector, effectorSim));
        Constants.OperatorConstants.driverController.b().onTrue(new Eject(effector, effectorSim, elevatorSim, drivetrain, bargeController));
        Constants.OperatorConstants.driverController.x().onTrue(new SimulateAlgaeIntake(effector, effectorSim, elevatorSim, drivetrain));
        Constants.OperatorConstants.operatorController.a().onTrue(new SimulateCoralIntakeExtra(extraDriver, effectorSimExtra));
        Constants.OperatorConstants.operatorController.b().onTrue(new EjectExtra(effector, effectorSimExtra, elevatorSimExtra, extraDriver, bargeController));
        Constants.OperatorConstants.operatorController.x().onTrue(new SimulateAlgaeIntakeExtra(effector, effectorSimExtra, elevatorSimExtra, extraDriver));
        
        // Pathfinding control
        Constants.OperatorConstants.driverController.leftBumper().onTrue(new AlignToBranch(drivetrain, "Left").onlyIf(() -> effector.simulatedBeamBreak));
        Constants.OperatorConstants.driverController.rightBumper().onTrue(new AlignToBranch(drivetrain, "Right").onlyIf(() -> effector.simulatedBeamBreak));
        Constants.OperatorConstants.driverController.y().onTrue(new AlignToAlgae(drivetrain).onlyIf(() -> !effectorSim.hasAlgae));
        
        // Elevator Automatic Control    
        Constants.OperatorConstants.driverController.povUp().onTrue(elevatorSim.setState(ElevatorState.L3));
        Constants.OperatorConstants.driverController.povLeft().onTrue(elevatorSim.setState(ElevatorState.L2));
        Constants.OperatorConstants.driverController.povRight().onTrue(elevatorSim.setState(ElevatorState.L4));
        Constants.OperatorConstants.driverController.povDown().onTrue(elevatorSim.setState(ElevatorState.GROUND));

        Constants.OperatorConstants.operatorController.povUp().onTrue(elevatorSimExtra.simulateSetpoint(64));
        Constants.OperatorConstants.operatorController.povLeft().onTrue(elevatorSimExtra.simulateSetpoint(40));
        Constants.OperatorConstants.operatorController.povRight().onTrue(elevatorSimExtra.simulateSetpoint(100));
        Constants.OperatorConstants.operatorController.povDown().onTrue(elevatorSimExtra.simulateSetpoint(0));

        Constants.OperatorConstants.driverController.leftTrigger().onTrue(elevatorSim.setState(ElevatorState.MANUAL));
        Constants.OperatorConstants.driverController.rightTrigger().onTrue(elevatorSim.setState(ElevatorState.MANUAL));
        Constants.OperatorConstants.operatorController.leftTrigger().onTrue(Commands.runOnce(() -> elevatorSimExtra.manualControl = true));
        Constants.OperatorConstants.operatorController.rightTrigger().onTrue(Commands.runOnce(() -> elevatorSimExtra.manualControl = true));
        
        // Fancy logging stuff that fills all the storage on the RIO
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}