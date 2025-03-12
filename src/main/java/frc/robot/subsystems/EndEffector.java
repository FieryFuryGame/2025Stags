package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    TalonFX effectorWheels = new TalonFX(Constants.EndEffectorConstants.EffectorWheelsID, "Canivore");
    TalonFX effectorPivot = new TalonFX(Constants.EndEffectorConstants.EffectorPivotID, "Canivore");
    public DigitalInput beamBreak = new DigitalInput(0);

    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);
    public boolean pivotDown = true; // When turned on, the pivot should be down immediately.
    public BooleanSupplier checkBeam = () -> !beamBreak.get();
    
    public EndEffector() {
        // This is very useful. Does a lot.
        setMotorSettings();
        effectorPivot.setPosition(0.0);
        effectorPivot.setControl(positionVoltage.withPosition(effectorPivot.getPosition().getValueAsDouble()).withSlot(0));
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
        // mmBConfig.MotionMagicJerk = Constants.ElevatorConstants.bmmJerk;
        effectorPivot.getConfigurator().apply(pivotConfig);
        effectorPivot.getConfigurator().apply(mmpivotConfig);
        effectorPivot.setNeutralMode(NeutralModeValue.Brake);
        effectorWheels.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command setWheelVoltageCommand(double power) {
        return runOnce(() -> effectorWheels.setVoltage(power));
    }

    public void setWheelVoltage(double power) {
        effectorWheels.setVoltage(power);
    }

    public Command setPivotVoltage(double power) {
        return runOnce(() -> effectorPivot.setVoltage(power));
    }

    public Command useMotionMagicCommand(double position) {
        return runOnce(() -> effectorPivot.setControl(positionVoltage.withPosition(position).withSlot(0)));
    }

    public Command intakeWithWait(double seconds, double power) {
        return new WaitCommand(seconds).andThen(setWheelVoltageCommand(power));
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
        SmartDashboard.putBoolean("coralLoadedInEffector", isCoralLoaded());
    }
    
}