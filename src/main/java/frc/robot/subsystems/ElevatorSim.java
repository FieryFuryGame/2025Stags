package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSim extends SubsystemBase {
    Pose3d affector = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    Pose3d firstStage = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    Pose3d secondStage = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    
    public double percentageUp = 0;
    double goalPercentage = 0;
    public boolean manualControl = true;

    public BooleanSupplier isL1 = () -> percentageUp < 2.0;
    public BooleanSupplier isL4 = () -> percentageUp > 96.5;

    InterpolatingDoubleTreeMap affectorPos = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap firstStagePos = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap secondStagePos = new InterpolatingDoubleTreeMap();

    StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("ElevatorMechanism", Pose3d.struct).publish();

    public ElevatorSim() {
        affectorPos.put(0.0, 0.0);
        affectorPos.put(100.0, 1.73);
        firstStagePos.put(0.0, 0.0);
        firstStagePos.put(26.0, 0.0);
        firstStagePos.put(100.0, 1.28);
        secondStagePos.put(0.0, 0.0);
        secondStagePos.put(62.0, 0.0);
        secondStagePos.put(100.0, 0.65);
    }

    public void updatePoses() {
        if (DriverStation.isEnabled()) {
            if (manualControl) {
                if (Constants.OperatorConstants.driverController.leftTrigger().getAsBoolean() && percentageUp > 0) {
                    percentageUp -= 1;
                }
                if (Constants.OperatorConstants.driverController.rightTrigger().getAsBoolean() && percentageUp < 100) {
                    percentageUp += 1;
                }
            } else {
                if (percentageUp % 2 == 1) {
                    percentageUp -= 1;
                }
                if (goalPercentage < percentageUp) {
                    percentageUp -= 2;
                } else if (goalPercentage > percentageUp) {
                    percentageUp += 2;
                }
            }
        }
        affector = new Pose3d(0, 0, affectorPos.get(percentageUp), new Rotation3d(0, 0, 0));
        firstStage = new Pose3d(0, 0, firstStagePos.get(percentageUp), new Rotation3d(0, 0, 0));
        secondStage = new Pose3d(0, 0, secondStagePos.get(percentageUp), new Rotation3d(0, 0, 0));
    }

    public void moveElevatorUp() {
        manualControl = true;
        if (percentageUp < 100) {
            percentageUp += 1;
        }
    }

    public void moveElevatorDown() {
        manualControl = true;
        if (percentageUp > 0) {
            percentageUp -= 1;
        } 
    }

    public void createSetpoint(double goal) {
        manualControl = false;
        goalPercentage = goal;
    }

    public Command simulateSetpoint(double goal) {
        return runOnce(() -> createSetpoint(goal));
    }

    public Command down() {
        return runOnce(() -> moveElevatorDown());
    }

    public Command up() {
        return runOnce(() -> moveElevatorUp());
    }

    @Override
    public void periodic() {
        updatePoses();
        publisher.set(new Pose3d[]{affector, firstStage, secondStage});
        SmartDashboard.putNumber("ElevatorPercentage", percentageUp);
    }
}
