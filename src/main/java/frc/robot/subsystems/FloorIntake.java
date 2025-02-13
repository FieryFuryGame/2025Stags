package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorIntakeConstants;

public class FloorIntake extends SubsystemBase {

    TalonFX leftPivot = new TalonFX(FloorIntakeConstants.leftPivotMotorID);
    TalonFX leftWheels = new TalonFX(FloorIntakeConstants.leftWheelsMotorID);
    public boolean leftDown = false;

    TalonFX rightPivot = new TalonFX(FloorIntakeConstants.rightPivotMotorID);
    TalonFX rightWheels = new TalonFX(FloorIntakeConstants.rightWheelsMotorID);
    public boolean rightDown = false;
    
    public FloorIntake() {
        setMotorSettings();
    }

    public void setMotorSettings() {

        // Settings For Left Pivot
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        Slot0Configs leftSlot = leftConfig.Slot0;
        leftSlot.kS = FloorIntakeConstants.leftkS;
        leftSlot.kV = FloorIntakeConstants.leftkV;
        leftSlot.kA = FloorIntakeConstants.leftkA;
        leftSlot.kP = FloorIntakeConstants.leftkP;
        leftSlot.kI = FloorIntakeConstants.leftkI;
        leftSlot.kD = FloorIntakeConstants.leftkD;

        MotionMagicConfigs mmLeftConfig = leftConfig.MotionMagic;
        mmLeftConfig.MotionMagicCruiseVelocity = FloorIntakeConstants.leftmmCruiseVelocity;
        mmLeftConfig.MotionMagicAcceleration = FloorIntakeConstants.leftmmAccel;
        // mmLeftConfig.MotionMagicJerk = FloorIntakeConstants.leftmmJerk;
        leftPivot.getConfigurator().apply(leftConfig);

        // Settings For Right Pivot
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        Slot0Configs rightSlot = rightConfig.Slot0;
        rightSlot.kS = FloorIntakeConstants.rightkS;
        rightSlot.kV = FloorIntakeConstants.rightkV;
        rightSlot.kA = FloorIntakeConstants.rightkA;
        rightSlot.kP = FloorIntakeConstants.rightkP;
        rightSlot.kI = FloorIntakeConstants.rightkI;
        rightSlot.kD = FloorIntakeConstants.rightkD;

        MotionMagicConfigs mmRightConfig = rightConfig.MotionMagic;
        mmRightConfig.MotionMagicCruiseVelocity = FloorIntakeConstants.rightmmCruiseVelocity;
        mmRightConfig.MotionMagicAcceleration = FloorIntakeConstants.rightmmAccel;
        // mmRightConfig.MotionMagicJerk = FloorIntakeConstants.rightmmJerk;
        rightPivot.getConfigurator().apply(rightConfig);
    }

    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);

    public Command leftToggle() {
        if (leftDown) {
            return leftUp();
        } else {
            return leftDown();
        }
    }

    public Command rightToggle() {
        if (rightDown) {
            return rightUp();
        } else {
            return rightDown();
        }
    }

    // Currently, these set the position to stay at as the position they have already.
    public Command leftUp() {
        return runOnce(() -> {
            leftPivot.setControl(positionVoltage.withPosition(leftPivot.getPosition().getValueAsDouble())); // Replace with up position
            leftDown = false;
        });
    }

    public Command leftDown() {
        return runOnce(() -> {
            leftPivot.setControl(positionVoltage.withPosition(leftPivot.getPosition().getValueAsDouble())); // Replace with down position
            leftDown = true;
        });
    }

    public Command rightUp() {
        return runOnce(() -> {
            rightPivot.setControl(positionVoltage.withPosition(leftPivot.getPosition().getValueAsDouble())); // Replace with up position
            rightDown = false;
        });
    }

    public Command rightDown() {
        return runOnce(() -> {
            rightPivot.setControl(positionVoltage.withPosition(leftPivot.getPosition().getValueAsDouble())); // Replace with down position
            rightDown = true;
        });
    }

    public Command powerLeftIntake(double power) {
        return runOnce(() -> leftWheels.setVoltage(power));
    }

    public Command powerRightIntake(double power) {
        return runOnce(() -> rightWheels.setVoltage(power));
    }


}
