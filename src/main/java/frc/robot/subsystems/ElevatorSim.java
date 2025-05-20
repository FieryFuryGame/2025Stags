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
    double elevatorSpeed = 4;

    public enum ElevatorState {
        GROUND(true, 0, 4, "kGroundSetpoint"),
        GROUNDSLOW(true, 0, 1, "kGroundSetpointSlow"),
        L2(true, 40, 4, "kL2Setpoint"),
        L2SLOW(true, 40, 1, "kL2SetpointSlow"),
        L3(true, 64, 4, "kL3Setpoint"),
        L3SLOW(true, 64, 1, "kL3SetpointSlow"),
        L4(true, 100, 4, "kL4Setpoint"),
        L4SLOW(true, 100, 1, "kL4SetpointSlow"),
        MANUAL(false, 0, 4, "kManualControl");

        boolean isSetpoint;
        double setpoint;
        double speed;
        String stateTitle;

        ElevatorState(boolean isSetpoint, double setpoint, double speed, String stateTitle) {
            this.isSetpoint = isSetpoint;
            this.setpoint = setpoint;
            this.speed = speed;
            this.stateTitle = stateTitle;
        }

        public boolean isSetpoint() {
            return isSetpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }

        public String getTitle() {
            return stateTitle;
        }

        public double getSpeed() {
            return speed;
        }

    }
    
    public double percentageUp = 0;
    double goalPercentage = 0;
    public ElevatorState state = ElevatorState.MANUAL;

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
            if (!state.isSetpoint()) {
                if (Constants.OperatorConstants.driverController.leftTrigger().getAsBoolean() && percentageUp > 0) {
                    percentageUp -= 1;
                }
                if (Constants.OperatorConstants.driverController.rightTrigger().getAsBoolean() && percentageUp < 100) {
                    percentageUp += 1;
                }
            } else {
                if (percentageUp % 2 == 1) {
                    percentageUp += 1;
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
        SmartDashboard.putNumber("ElevatorPercentage", percentageUp);
        SmartDashboard.putString("ElevatorState", state.getTitle());
    }
}
