package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    TalonFX effectorWheels = new TalonFX(Constants.EndEffectorConstants.EffectorWheelsID, "Canivore");
    TalonFX effectorPivot = new TalonFX(Constants.EndEffectorConstants.EffectorPivotID, "Canivore");
    TalonFX conveyor = new TalonFX(Constants.EndEffectorConstants.ConveyorID, "Canivore");
    public DigitalInput beamBreak = new DigitalInput(0);

    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);
    public boolean pivotDown = true; // When turned on, the pivot should be down immediately.
    public BooleanSupplier checkBeam = () -> !beamBreak.get();

    public boolean simulatedBeamBreak = true;
    
    public int simulatedBlueScore = 0;
    public int blueProcessorAlgae = 0, blueL4Coral = 0, blueL3Coral = 0, blueL2Coral = 0;
    public int simulatedRedScore = 0;
    public int redProcessorAlgae = 0, redL4Coral = 0, redL3Coral = 0, redL2Coral = 0;
    public boolean redCoralRP = false, redCoopertitionRP = false;
    public boolean blueCoralRP = false, blueCoopertitionRP = false;

    public List<Pose3d> reefCoral = new ArrayList<>();
    Pose3d[] reefArray = new Pose3d[]{};

    public List<Pose3d> algaePositions = new ArrayList<>();
    Pose3d[] algaeArray = new Pose3d[]{};

    StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("CoralPositions", Pose3d.struct).publish();
    
    StructArrayPublisher<Pose3d> algaePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("AlgaePositions", Pose3d.struct).publish();
    
    public EndEffector() {
        setMotorSettings();
        effectorPivot.setPosition(0.0);
        effectorPivot.setControl(positionVoltage.withPosition(effectorPivot.getPosition().getValueAsDouble()).withSlot(0));
        SmartDashboard.putData("Reset Simulation", runOnce(() -> 
            {
                clearArray();
                resetAlgaeArray();
                simulatedBlueScore = 0;
                simulatedRedScore = 0;
                blueProcessorAlgae = 0;
                blueL2Coral = 0;
                blueL3Coral = 0;
                blueL4Coral = 0;
                redProcessorAlgae = 0;
                redL2Coral = 0;
                redL3Coral = 0;
                redL4Coral = 0;
                simulatedBeamBreak = true;
                redCoralRP = false;
                redCoopertitionRP = false;
                blueCoralRP = false;
                blueCoopertitionRP = false;
            }).ignoringDisable(true)
        );
        setupAlgaePositions();
        updateAlgaeArray();
    }

    public void setMotorSettings() {

        // Settings For Motor B
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        Slot0Configs pivotSlot = pivotConfig.Slot0;
        pivotSlot.kS = Constants.EndEffectorConstants.pivotkS;
        pivotSlot.kV = Constants.EndEffectorConstants.pivotkV;
        pivotSlot.kA = Constants.EndEffectorConstants.pivotkA;
        pivotSlot.kP = Constants.EndEffectorConstants.pivotkP;
        pivotSlot.kI = Constants.EndEffectorConstants.pivotkI;
        pivotSlot.kD = Constants.EndEffectorConstants.pivotkD;
        
        MotionMagicConfigs mmpivotConfig = pivotConfig.MotionMagic;
        mmpivotConfig.MotionMagicCruiseVelocity = Constants.EndEffectorConstants.pivotmmCruiseVelocity;
        mmpivotConfig.MotionMagicAcceleration = Constants.EndEffectorConstants.pivotmmAccel;
        
        effectorPivot.getConfigurator().apply(pivotConfig);
        effectorPivot.getConfigurator().apply(mmpivotConfig);
        effectorPivot.setNeutralMode(NeutralModeValue.Brake);
        effectorWheels.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command setWheelVoltageCommand(double power) {
        return runOnce(() -> effectorWheels.setVoltage(power));
    }

    public void setupAlgaePositions() {
        algaePositions.add(new Pose3d(5.179, 4.025, 0.9, Rotation3d.kZero));
        algaePositions.add(new Pose3d(4.831, 3.42, 1.3, Rotation3d.kZero));
        algaePositions.add(new Pose3d(4.136, 3.42, 0.9, Rotation3d.kZero));
        algaePositions.add(new Pose3d(3.812, 4.025, 1.3, Rotation3d.kZero));
        algaePositions.add(new Pose3d(4.148, 4.618, 0.9, Rotation3d.kZero));
        algaePositions.add(new Pose3d(4.832, 4.618, 1.3, Rotation3d.kZero));
    }

    public void updateArray() {
        if (!reefCoral.isEmpty()) {
            int length = reefCoral.size();
            Pose3d[] poses = new Pose3d[length];
            for (int i = 0; i < length; i++) {
                poses[i] = reefCoral.get(i);
            }
            reefArray = poses;
        }
    }

    public void updateAlgaeArray() {
        if (!algaePositions.isEmpty()) {
            int length = algaePositions.size();
            Pose3d[] poses = new Pose3d[length];
            for (int i = 0; i < length; i++) {
                poses[i] = algaePositions.get(i);
            }
            algaeArray = poses;
        }
    }

    public void clearArray() {
        reefCoral.clear();
        reefArray = new Pose3d[]{};
    }

    public void resetAlgaeArray() {
        algaePositions.clear();
        setupAlgaePositions();
        updateAlgaeArray();
    }

    public void setWheelVoltage(double power) {
        effectorWheels.setVoltage(power);
    }
    
    public Command setConveyorVoltageCommand(double power) {
        return runOnce(() -> conveyor.setVoltage(power));
    }

    public void setConveyorVoltage(double power) {
        conveyor.setVoltage(power);
    }

    public Command setPivotVoltage(double power) {
        return runOnce(() -> effectorPivot.setVoltage(power));
    }

    public Command useMotionMagicCommand(double position) {
        return runOnce(() -> effectorPivot.setControl(positionVoltage.withPosition(position).withSlot(0)));
    }

    public boolean isCoralLoaded() {
        return !beamBreak.get(); // True if coral loaded, false if not
    }

    public Command runEffectorPivot() {
        if (pivotDown) {
            return useMotionMagicCommand(7).alongWith(Commands.runOnce(() -> pivotDown = false).andThen(() -> System.out.println("[Effector] Pivot Down")));
        } else {
            return useMotionMagicCommand(0).alongWith(Commands.runOnce(() -> pivotDown = true).andThen(() -> System.out.println("[Effector] Pivot Up")));
        }
    }

    public void useMotionMagic(double pos) {
        effectorPivot.setControl(positionVoltage.withPosition(pos).withSlot(0));
    }

    public Command intake() {
        if (isCoralLoaded()) {
            return setWheelVoltageCommand(0);
        } else {
            return setWheelVoltageCommand(-6);
        }
    }

    public Command eject() {
        if (!isCoralLoaded()) {
            return setWheelVoltageCommand(0);
        } else {
            return setWheelVoltageCommand(-6);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("endEffectorPivotPos", effectorPivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("conveyorRPM", conveyor.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("coralLoadedInEffector", simulatedBeamBreak);
        SmartDashboard.putNumber("Blue Alliance Score", simulatedBlueScore);
        SmartDashboard.putNumber("Red Alliance Score", simulatedRedScore);

        publisher.set(reefArray);
        algaePublisher.set(algaeArray);
    }
    
}