package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
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
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);

    public void setLevelOne() {
        elevatorMotorA.setControl(positionVoltage.withPosition(10));
        elevatorMotorB.setControl(positionVoltage.withPosition(10));
    }

    public void setLevelTwo() {
        elevatorMotorA.setControl(positionVoltage.withPosition(30));
        elevatorMotorB.setControl(positionVoltage.withPosition(30));
    }

    public void setLevelThree() {
        elevatorMotorA.setControl(positionVoltage.withPosition(50));
        elevatorMotorB.setControl(positionVoltage.withPosition(50));
    }

    public void setLevelFour() {
        elevatorMotorA.setControl(positionVoltage.withPosition(70));
        elevatorMotorB.setControl(positionVoltage.withPosition(70));
    }

    public Command setVoltage(double power) {
        return runOnce(() -> {
            elevatorMotorA.setVoltage(power);
            elevatorMotorB.setVoltage(power);
        });
    }
    

}
