package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BargeController extends SubsystemBase {

    public List<Pose3d> bargeList = new ArrayList<>();
    public Pose3d[] bargeArray = new Pose3d[]{};

    public Pose3d shotBlueAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
    public Pose3d shotRedAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
    double blueTime = 0.0;
    double redTime = 0.0;

    StructArrayPublisher<Pose3d> bargePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Barge Algae", Pose3d.struct).publish();

    StructPublisher<Pose3d> shotBlueAlgaePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Shot Blue Algae", Pose3d.struct).publish();
    StructPublisher<Pose3d> shotRedAlgaePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Shot Red Algae", Pose3d.struct).publish();
    
    public BargeController() {
        
    }

    @Override
    public void periodic() {
        bargePublisher.set(bargeArray);

        shotBlueAlgaePublisher.set(shotBlueAlgae);
        shotRedAlgaePublisher.set(shotRedAlgae);
    }

}
