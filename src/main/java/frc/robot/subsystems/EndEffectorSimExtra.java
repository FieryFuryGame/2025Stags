package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSimExtra extends SubsystemBase {
    ElevatorSimExtra elevatorSimExtra;
    ExtraDriver drivetrain;
    public boolean simulatedBeamBreak = true;

    InterpolatingDoubleTreeMap coralHeight = new InterpolatingDoubleTreeMap();

    StructPublisher<Pose3d> heldCoral = NetworkTableInstance.getDefault()
        .getStructTopic("HeldCoralExtra", Pose3d.struct).publish();
    StructPublisher<Pose3d> heldAlgae = NetworkTableInstance.getDefault()
        .getStructTopic("HeldAlgaeExtra", Pose3d.struct).publish();

    public boolean hasAlgae = false;
    
    public EndEffectorSimExtra(ElevatorSimExtra elevatorSimExtra, ExtraDriver drivetrain) {
        this.elevatorSimExtra = elevatorSimExtra;
        this.drivetrain = drivetrain;

        coralHeight.put(0.0, 0.21);
        coralHeight.put(100.0, 1.89);
    }

    @Override
    public void periodic() {
        Pose3d coralPose = new Pose3d(drivetrain.getState().Pose);
        if (simulatedBeamBreak) {
            coralPose = coralPose.transformBy(new Transform3d(0.29, 0.0, coralHeight.get(elevatorSimExtra.percentageUp), new Rotation3d(0, 60, 0)));
        } else {
            coralPose = new Pose3d(0, 0, -3, new Rotation3d(0, 0, 0));
        }
        Pose3d algaePose = new Pose3d(drivetrain.getState().Pose);
        if (hasAlgae) {
            algaePose = algaePose.transformBy(new Transform3d(0.29, 0.0, 0.05 + coralHeight.get(elevatorSimExtra.percentageUp), Rotation3d.kZero));
        } else {
            algaePose = new Pose3d(0, 0, -5, Rotation3d.kZero);
        }
        heldCoral.set(coralPose);
        heldAlgae.set(algaePose);
    }

}
