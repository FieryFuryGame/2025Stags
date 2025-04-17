package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSim extends SubsystemBase {

    EndEffector effector;
    ElevatorSim elevatorSim;
    CommandSwerveDrivetrain drivetrain;

    InterpolatingDoubleTreeMap coralHeight = new InterpolatingDoubleTreeMap();

    Pose3d algae1 = new Pose3d(5.179, 4.025, 0.9, Rotation3d.kZero), 
    algae2 = new Pose3d(4.831, 3.42, 1.3, Rotation3d.kZero), 
    algae3 = new Pose3d(4.136, 3.42, 0.9, Rotation3d.kZero), 
    algae4 = new Pose3d(3.812, 4.025, 1.3, Rotation3d.kZero), 
    algae5 = new Pose3d(4.148, 4.618, 0.9, Rotation3d.kZero), 
    algae6 = new Pose3d(4.832, 4.618, 1.3, Rotation3d.kZero);

    boolean algae1taken = false, algae2taken = false, algae3taken = false, algae4taken = false, algae5taken = false, algae6taken = false;

    Pose3d[] algaeArray = {algae1, algae2, algae3, algae4, algae5, algae6};

    StructPublisher<Pose3d> heldCoral = NetworkTableInstance.getDefault()
        .getStructTopic("HeldCoral", Pose3d.struct).publish();

    StructArrayPublisher<Pose3d> reefAlgae = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Algae Positions", Pose3d.struct).publish();

    StructPublisher<Pose3d> heldAlgae = NetworkTableInstance.getDefault()
    .getStructTopic("HeldAlgae", Pose3d.struct).publish();
    
    public EndEffectorSim(EndEffector effector, ElevatorSim elevatorSim, CommandSwerveDrivetrain drivetrain) {
        this.effector = effector;
        this.elevatorSim = elevatorSim;
        this.drivetrain = drivetrain;

        coralHeight.put(0.0, 0.21);
        coralHeight.put(100.0, 1.89);
    }

    public void setupAlgae() {

    }

    @Override
    public void periodic() {
        Pose3d coralPose = new Pose3d(drivetrain.getState().Pose);
        if (effector.simulatedBeamBreak) {
            coralPose = coralPose.transformBy(new Transform3d(0.29, 0.0, coralHeight.get(elevatorSim.percentageUp), new Rotation3d(0, 60, 0)));
        } else {
            coralPose = new Pose3d(0, 0, -3, new Rotation3d(0, 0, 0));
        }
        heldCoral.set(coralPose);
        reefAlgae.set(algaeArray);
    }

}
