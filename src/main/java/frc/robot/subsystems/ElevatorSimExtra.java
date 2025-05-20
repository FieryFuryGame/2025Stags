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
import frc.robot.subsystems.ElevatorSim.ElevatorState;

public class ElevatorSimExtra extends SubsystemBase {
    Pose3d affector = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    Pose3d firstStage = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    Pose3d secondStage = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    
    public double percentageUp = 0;
    double goalPercentage = 0;
    double elevatorSpeed = 4;

    public BooleanSupplier isL1 = () -> percentageUp < 2.0;
    public BooleanSupplier isL4 = () -> percentageUp > 96.5;

    ElevatorState state = ElevatorState.MANUAL;

    InterpolatingDoubleTreeMap affectorPos = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap firstStagePos = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap secondStagePos = new InterpolatingDoubleTreeMap();

    StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("ElevatorMechanismExtra", Pose3d.struct).publish();

    public ElevatorSimExtra() {
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
            if (!state.isSetpoint) {
                if (Constants.OperatorConstants.operatorController.leftTrigger().getAsBoolean() && percentageUp > 0) {
                    percentageUp -= 1;
                }
                if (Constants.OperatorConstants.operatorController.rightTrigger().getAsBoolean() && percentageUp < 100) {
                    percentageUp += 1;
                }
            } else {
                if (percentageUp % 2 == 1) {
                    percentageUp -= 1;
                }
                if (goalPercentage < percentageUp) {
                    percentageUp -= elevatorSpeed;
                } else if (goalPercentage > percentageUp) {
                    percentageUp += elevatorSpeed;
                }
            }
        }
        affector = new Pose3d(0, 0, affectorPos.get(percentageUp), new Rotation3d(0, 0, 0));
        firstStage = new Pose3d(0, 0, firstStagePos.get(percentageUp), new Rotation3d(0, 0, 0));
        secondStage = new Pose3d(0, 0, secondStagePos.get(percentageUp), new Rotation3d(0, 0, 0));
    }

    public void handleState(ElevatorState state) {
        this.state = state;

        if (state.isSetpoint()) {
            elevatorSpeed = state.getSpeed();
            goalPercentage = state.getSetpoint();
        }
    }

    public Command setState(ElevatorState state) {
        return runOnce(() -> handleState(state));
    }

    @Override
    public void periodic() {
        updatePoses();
        publisher.set(new Pose3d[]{affector, firstStage, secondStage});
        SmartDashboard.putNumber("ElevatorPercentageExtra", percentageUp);
        SmartDashboard.putString("ElevatorStateExtra", state.getTitle());
    }
}
