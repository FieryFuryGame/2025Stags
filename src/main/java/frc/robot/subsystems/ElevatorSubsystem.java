package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    TalonFX elevatorMotorA = new TalonFX(Constants.ElevatorConstants.ElevatorMotorAID);
    TalonFX elevatorMotorB = new TalonFX(Constants.ElevatorConstants.ElevatorMotorBID);

    public ElevatorSubsystem() {
        setMotorSettings();
    }

    public void setMotorSettings() {

        // Settings For Motor A
        TalonFXConfiguration motorAConfig = new TalonFXConfiguration();
        Slot0Configs motorASlot = motorAConfig.Slot0;
        motorASlot.kS = Constants.ElevatorConstants.akS;
        motorASlot.kV = Constants.ElevatorConstants.akV;
        motorASlot.kA = Constants.ElevatorConstants.akA;
        motorASlot.kP = Constants.ElevatorConstants.akP;
        motorASlot.kI = Constants.ElevatorConstants.akI;
        motorASlot.kD = Constants.ElevatorConstants.akD;

        MotionMagicConfigs mmAConfig = motorAConfig.MotionMagic;
        mmAConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.ammCruiseVelocity;
        mmAConfig.MotionMagicAcceleration = Constants.ElevatorConstants.ammAccel;
        // mmAConfig.MotionMagicJerk = Constants.ElevatorConstants.ammJerk;
        elevatorMotorA.getConfigurator().apply(motorAConfig);

        // Settings For Motor B
        TalonFXConfiguration motorBConfig = new TalonFXConfiguration();
        Slot0Configs motorBSlot = motorBConfig.Slot0;
        motorBSlot.kS = Constants.ElevatorConstants.bkS;
        motorBSlot.kV = Constants.ElevatorConstants.bkV;
        motorBSlot.kA = Constants.ElevatorConstants.bkA;
        motorBSlot.kP = Constants.ElevatorConstants.bkP;
        motorBSlot.kI = Constants.ElevatorConstants.bkI;
        motorBSlot.kD = Constants.ElevatorConstants.bkD;

        MotionMagicConfigs mmBConfig = motorBConfig.MotionMagic;
        mmBConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.bmmCruiseVelocity;
        mmBConfig.MotionMagicAcceleration = Constants.ElevatorConstants.bmmAccel;
        // mmBConfig.MotionMagicJerk = Constants.ElevatorConstants.bmmJerk;
        elevatorMotorB.getConfigurator().apply(motorBConfig);
    }

    // povUp L4, povLeft L3, povRight L2, povDown L1

    public void setLevelOne() {

    }

    public void setLevelTwo() {

    }

    public void setLevelThree() {

    }

    public void setLevelFour() {
        
    }

}
