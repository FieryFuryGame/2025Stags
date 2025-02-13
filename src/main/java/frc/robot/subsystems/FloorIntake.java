package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.FloorIntakeConstants;

public class FloorIntake {

    TalonFX leftPivot = new TalonFX(FloorIntakeConstants.leftPivotMotorID);
    TalonFX leftWheels = new TalonFX(FloorIntakeConstants.leftWheelsMotorID);

    TalonFX rightPivot = new TalonFX(FloorIntakeConstants.rightPivotMotorID);
    TalonFX rightWheels = new TalonFX(FloorIntakeConstants.rightWheelsMotorID);
    
    public FloorIntake() {
        
    }

    public void setMotorSettings() {

        // Settings For Left Pivot
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        Slot0Configs leftSlot = leftConfig.Slot0;
        leftSlot.kS = Constants.ElevatorConstants.akS;
        leftSlot.kV = Constants.ElevatorConstants.akV;
        leftSlot.kA = Constants.ElevatorConstants.akA;
        leftSlot.kP = Constants.ElevatorConstants.akP;
        leftSlot.kI = Constants.ElevatorConstants.akI;
        leftSlot.kD = Constants.ElevatorConstants.akD;

        MotionMagicConfigs mmLeftConfig = leftConfig.MotionMagic;
        mmLeftConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.ammCruiseVelocity;
        mmLeftConfig.MotionMagicAcceleration = Constants.ElevatorConstants.ammAccel;
        // mmAConfig.MotionMagicJerk = Constants.ElevatorConstants.ammJerk;
        leftPivot.getConfigurator().apply(leftConfig);

        // Settings For Right Pivot
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        Slot0Configs rightSlot = rightConfig.Slot0;
        rightSlot.kS = Constants.ElevatorConstants.bkS;
        rightSlot.kV = Constants.ElevatorConstants.bkV;
        rightSlot.kA = Constants.ElevatorConstants.bkA;
        rightSlot.kP = Constants.ElevatorConstants.bkP;
        rightSlot.kI = Constants.ElevatorConstants.bkI;
        rightSlot.kD = Constants.ElevatorConstants.bkD;

        MotionMagicConfigs mmRightConfig = rightConfig.MotionMagic;
        mmRightConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.bmmCruiseVelocity;
        mmRightConfig.MotionMagicAcceleration = Constants.ElevatorConstants.bmmAccel;
        // mmBConfig.MotionMagicJerk = Constants.ElevatorConstants.bmmJerk;
        rightPivot.getConfigurator().apply(rightConfig);
    }


}
