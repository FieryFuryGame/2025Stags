package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BargeController extends SubsystemBase {

    EndEffector effector;

    public Pose3d shotBlueAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
    public Pose3d shotRedAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
    double blueTime = 0.0;
    double redTime = 0.0;
    boolean isShootingBlue = false;
    boolean isShootingRed = false;

    StructArrayPublisher<Pose3d> bargePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Barge Algae", Pose3d.struct).publish();

    StructPublisher<Pose3d> shotBlueAlgaePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Shot Blue Algae", Pose3d.struct).publish();
    StructPublisher<Pose3d> shotRedAlgaePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Shot Red Algae", Pose3d.struct).publish();
    
    public BargeController(EndEffector effector) {
        this.effector = effector;
    }

    public void shootBlueAlgae() {
        blueTime = 0.0;
        isShootingBlue = true;
    }

    public void shootRedAlgae() {
        redTime = 0.0;
        isShootingRed = true;
    }

    @Override
    public void periodic() {
        if (blueTime >= 1.0) {
            isShootingBlue = false;
            blueTime = 0.0;
            shotBlueAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
            effector.simulatedBlueScore += 4;
        }
        if (redTime >= 1.0) {
            isShootingRed = false;
            redTime = 0.0;
            shotRedAlgae = new Pose3d(0, 0, -5, Rotation3d.kZero);
            effector.simulatedRedScore += 4;
        }

        if (isShootingBlue) {
            blueTime += 0.02;
            shotBlueAlgae = new Pose3d(11.364, 8.649, 2.5, Rotation3d.kZero).interpolate(new Pose3d(8.787, 6.165, 2.12, new Rotation3d(0, -90, 0)), blueTime);
        }
        if (isShootingRed) {
            redTime += 0.02;
            shotRedAlgae = new Pose3d(5.970, -0.488, 2.5, Rotation3d.kZero).interpolate(new Pose3d(8.751, 1.909, 2.12, new Rotation3d(0, 90, 0)), redTime);
        }

        shotBlueAlgaePublisher.set(shotBlueAlgae);
        shotRedAlgaePublisher.set(shotRedAlgae);
    }

}
