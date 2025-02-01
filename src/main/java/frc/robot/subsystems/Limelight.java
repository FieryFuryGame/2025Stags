package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    String name;
    NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    NetworkTable table;

    CommandSwerveDrivetrain drivetrain;
    
    int tv;
    double tx;
    double ty;
    double ta;
    int tid;

    InterpolatingDoubleTreeMap areaMap = new InterpolatingDoubleTreeMap();

    public void setupMap() {
        areaMap.put(41.0, 8.25);
        areaMap.put(17.5, 14.0);
        areaMap.put(6.6, 24.0);
        areaMap.put(5.4, 26.625);
        areaMap.put(4.1, 31.125);
        areaMap.put(3.2, 35.375);
        areaMap.put(2.37, 41.5);
        areaMap.put(1.6, 50.375);
        areaMap.put(0.98, 65.625);
        areaMap.put(0.64, 81.3125);
        areaMap.put(0.3, 114.125);
    }

    public Limelight(String limelightName, CommandSwerveDrivetrain swerve) {
        setupMap();
        name = limelightName;
        drivetrain = swerve;
        table = tableInstance.getTable(limelightName);
    }

    public double getTargetDistanceMath() {

        double targetOffsetAngle_Vertical = ty;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0.343;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 8;

        // distance from the target to the floor
        double goalHeightInches = 12.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    

    @Override
    public void periodic() {
        tv = (int) table.getValue("tv").getDouble();
        tx = table.getValue("tx").getDouble();
        ty = table.getValue("ty").getDouble();
        tid = (int) table.getValue("tid").getDouble();
        ta = table.getValue("ta").getDouble();

        SmartDashboard.putNumber("td", areaMap.get(ta)); // Target Distance
        
    }

}
