package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Limelight extends SubsystemBase {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    String name;
    NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    NetworkTable table;

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon2;
    
    int tv;
    double tx;
    double ty;
    double ta;
    int tid;
    public int teamAdd = 0;
    public int rotateDirection = 0;
    int tagAngle;

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
        pigeon2 = drivetrain.getPigeon2();
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

    public Command rotateToTag() {
        return drivetrain.applyRequest(() ->
            drive.withRotationalRate(0.3 * rotateDirection * MaxAngularRate)).unless(() -> tagAngle == -1)
                .until(() -> MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees() + teamAdd, 0, 360) <= tagAngle + 2 && MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees() + teamAdd, 0, 360) > tagAngle - 2);
    }

    public void getTagAngle() {
        switch (tid) {
            case 6:
                tagAngle = Constants.TagConstants.Tag6Angle;
                break;
            case 7:
                tagAngle = Constants.TagConstants.Tag7Angle;
                break;
            case 8:
                tagAngle = Constants.TagConstants.Tag8Angle;
                break;
            case 9:
                tagAngle = Constants.TagConstants.Tag9Angle;
                break;
            case 10:
                tagAngle = Constants.TagConstants.Tag10Angle;
                break;
            case 11:
                tagAngle = Constants.TagConstants.Tag11Angle;
                break;
            case 17:
                tagAngle = Constants.TagConstants.Tag17Angle;
                break;
            case 18:
                tagAngle = Constants.TagConstants.Tag18Angle;
                break;
            case 19:
                tagAngle = Constants.TagConstants.Tag19Angle;
                break;
            case 20:
                tagAngle = Constants.TagConstants.Tag20Angle;
                break;
            case 21:
                tagAngle = Constants.TagConstants.Tag21Angle;
                break;
            case 22:
                tagAngle = Constants.TagConstants.Tag22Angle;
                break;
            default:
                System.out.println("[Limelight] Incorrect tag!");
                tagAngle = -1;
                break;
        }
    }

    public int decideRotationDirection() {
        if (tx < 0) {
            return 1;
        } else {
            return -1;
        }
    }

    @Override
    public void periodic() {
        tv = (int) table.getValue("tv").getDouble();
        tx = table.getValue("tx").getDouble();
        ty = table.getValue("ty").getDouble();
        tid = (int) table.getValue("tid").getDouble();
        ta = table.getValue("ta").getDouble();
        SmartDashboard.putNumber("tr", MathUtil.inputModulus(pigeon2.getRotation2d().getDegrees() + teamAdd, 0,360));
        SmartDashboard.putNumber("td", areaMap.get(ta)); // Target Distance
    }
}
