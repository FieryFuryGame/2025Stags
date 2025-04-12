package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSim extends SubsystemBase {

    EndEffector effector;
    ElevatorSim elevatorSim;
    CommandSwerveDrivetrain drivetrain;

    InterpolatingDoubleTreeMap coralHeight = new InterpolatingDoubleTreeMap();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("HeldCoral", Pose3d.struct).publish();
    
    public EndEffectorSim(EndEffector effector, ElevatorSim elevatorSim, CommandSwerveDrivetrain drivetrain) {
        this.effector = effector;
        this.elevatorSim = elevatorSim;
        this.drivetrain = drivetrain;

        coralHeight.put(0.0, 0.21);
        coralHeight.put(100.0, 1.89);
    }

    @Override
    public void periodic() {
        Pose3d coralPose = new Pose3d(drivetrain.getState().Pose);
        if (effector.simulatedBeamBreak) {
            coralPose = coralPose.transformBy(new Transform3d(0.29, 0.0, coralHeight.get(elevatorSim.percentageUp), new Rotation3d(0, 60, 0)));
        } else {
            coralPose = new Pose3d(0, 0, -3, new Rotation3d(0, 0, 0));
        }
        publisher.set(coralPose);
    }

}
