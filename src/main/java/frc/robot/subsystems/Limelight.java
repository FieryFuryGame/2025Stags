package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    String name;
    NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    NetworkTable table;

    CommandSwerveDrivetrain drivetrain;
    
    long tv;
    double tx;
    double ty;
    long tid;

    public Limelight(String limelightName, CommandSwerveDrivetrain swerve) {
        name = limelightName;
        drivetrain = swerve;
        table = tableInstance.getTable(limelightName);
    }

    @Override
    public void periodic() {
        tv = table.getValue("tv").getInteger();
        tx = table.getValue("tx").getDouble();
        ty = table.getValue("ty").getDouble();
        tid = table.getValue("tid").getInteger();
    }

}
