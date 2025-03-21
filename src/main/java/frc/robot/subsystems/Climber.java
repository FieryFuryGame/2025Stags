package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    TalonFX motorA = new TalonFX(Constants.ClimberConstants.ClimberMotorA, "Canivore");
    TalonFX motorB = new TalonFX(Constants.ClimberConstants.ClimberMotorB, "Canivore");
    
    public Climber() {
        motorA.setPosition(0);
        motorB.setPosition(0);
        motorB.setControl(new Follower(Constants.ClimberConstants.ClimberMotorA, true));
    }

    public void setMotorSettings() {

        // Settings For Motor B
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        Slot0Configs motorSlot = motorConfig.Slot0;
        motorSlot.kS = Constants.ClimberConstants.kS;
        motorSlot.kV = Constants.ClimberConstants.kV;
        motorSlot.kA = Constants.ClimberConstants.kA;
        motorSlot.kP = Constants.ClimberConstants.kP;
        motorSlot.kI = Constants.ClimberConstants.kI;
        motorSlot.kD = Constants.ClimberConstants.kD;
        motorSlot.kG = Constants.ClimberConstants.kG;

        Slot1Configs motorSlot2 = motorConfig.Slot1;
        motorSlot2.kS = 0.0;
        motorSlot2.kV = 0.0;
        motorSlot2.kA = 0.0;
        motorSlot2.kP = 0.0;
        motorSlot2.kI = 0.0;
        motorSlot2.kD = 0.0;
        motorSlot2.kG = 0.3;
        
        MotionMagicConfigs mmConfig = motorConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = Constants.ClimberConstants.mmCruiseVelocity;
        mmConfig.MotionMagicAcceleration = Constants.ClimberConstants.mmAccel;
        // mmConfig.MotionMagicJerk = Constants.ClimberConstants.mmJerk;
        motorA.getConfigurator().apply(motorConfig);
        motorA.getConfigurator().apply(motorSlot);
        motorA.getConfigurator().apply(motorSlot2);
        motorA.getConfigurator().apply(mmConfig);
        motorA.setNeutralMode(NeutralModeValue.Brake);

        motorB.getConfigurator().apply(motorConfig);
        motorB.getConfigurator().apply(motorSlot);
        motorB.getConfigurator().apply(motorSlot2);
        motorB.getConfigurator().apply(mmConfig);
        motorB.setNeutralMode(NeutralModeValue.Brake);
    }

    VelocityVoltage voltageOut = new VelocityVoltage(0).withSlot(1);
    MotionMagicVoltage setpointVoltage = new MotionMagicVoltage(0).withSlot(0);

    public void setVoltage(double power) {
        motorA.setControl(voltageOut.withSlot(1).withVelocity(0).withFeedForward(-power));
        motorB.setControl(voltageOut.withSlot(1).withVelocity(0).withFeedForward(-power));
    }

    public Command setVoltageCommand(double power) {
        return Commands.runOnce(() -> setVoltage(power));
    }

    public void moveToSetpoint(double pos) {
        motorA.setControl(setpointVoltage.withSlot(0).withPosition(pos));
        motorB.setControl(setpointVoltage.withSlot(0).withPosition(pos));
    }

    public Command moveToSetpointCommand(double pos) {
        return Commands.runOnce(() -> moveToSetpoint(pos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberPosA", motorA.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ClimberPosB", motorB.getPosition().getValueAsDouble());
    }

}
