package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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

public class ElevatorSubsystem extends SubsystemBase {
    
    TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.ElevatorMotorAID, "Canivore");
    TalonFX elevatorMotorB = new TalonFX(Constants.ElevatorConstants.ElevatorMotorBID, "Canivore");
    Follower followControl = new Follower(Constants.ElevatorConstants.ElevatorMotorAID, false);
    
    public BooleanSupplier isL1 = () -> elevatorMotor.getPosition().getValueAsDouble() < 2;
    public BooleanSupplier isL4 = () -> elevatorMotor.getPosition().getValueAsDouble() > 96.75;

    public enum ElevatorState {
        GROUND("kGround", 0, true),
        L2("kL2", 34, true),
        L3("kL3", 53, true),
        L4("kL4", 79, true),
        MANUAL("kManual", 0, false);

        String title;
        double position;
        boolean isSetpoint;

        ElevatorState(String title, double position, boolean isSetpoint) {
            this.title = title;
            this.position = position;
            this.isSetpoint = isSetpoint;
        }

        public String getTitle() {
            return this.title;
        }

        public double getPosition() {
            return this.position;
        }

        public boolean isSetpoint() {
            return this.isSetpoint;
        }
    }

    public ElevatorState state = ElevatorState.MANUAL;

    public ElevatorSubsystem() {
        setMotorSettings();
        elevatorMotor.setPosition(0.0, 1.0);
        elevatorMotorB.setPosition(0.0, 1.0);

        SmartDashboard.putData("Zero Elevator", runOnce(() -> {
            elevatorMotor.setPosition(0.0, 1.0);
            elevatorMotorB.setPosition(0.0, 1.0);
        }));
    }
    
    public void setMotorSettings() {

        // Settings For Motor B
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        Slot0Configs motorSlot = motorConfig.Slot0;
        motorSlot.kS = Constants.ElevatorConstants.kS;
        motorSlot.kV = Constants.ElevatorConstants.kV;
        motorSlot.kA = Constants.ElevatorConstants.kA;
        motorSlot.kP = Constants.ElevatorConstants.kP;
        motorSlot.kI = Constants.ElevatorConstants.kI;
        motorSlot.kD = Constants.ElevatorConstants.kD;
        motorSlot.kG = Constants.ElevatorConstants.kG;

        Slot1Configs motorSlot2 = motorConfig.Slot1;
        motorSlot2.kS = 0.0;
        motorSlot2.kV = 0.0;
        motorSlot2.kA = 0.0;
        motorSlot2.kP = 0.0;
        motorSlot2.kI = 0.0;
        motorSlot2.kD = 0.0;
        motorSlot2.kG = 0.3;
        
        MotionMagicConfigs mmConfig = motorConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = Constants.ElevatorConstants.mmCruiseVelocity;
        mmConfig.MotionMagicAcceleration = Constants.ElevatorConstants.mmAccel;
        // mmConfig.MotionMagicJerk = Constants.ElevatorConstants.bmmJerk;
        elevatorMotor.getConfigurator().apply(motorConfig);
        elevatorMotor.getConfigurator().apply(motorSlot);
        elevatorMotor.getConfigurator().apply(motorSlot2);
        elevatorMotor.getConfigurator().apply(mmConfig);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        elevatorMotorB.getConfigurator().apply(motorConfig);
        elevatorMotorB.getConfigurator().apply(motorSlot);
        elevatorMotorB.getConfigurator().apply(motorSlot2);
        elevatorMotorB.getConfigurator().apply(mmConfig);
        elevatorMotorB.setNeutralMode(NeutralModeValue.Brake);
    }

    // povUp L4, povLeft L3, povRight L2, povDown L1
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);
    VelocityVoltage voltageOut = new VelocityVoltage(0);

    public Command setVoltage(double power) {
        return runOnce(() -> {
            elevatorMotor.setControl(voltageOut.withSlot(1).withVelocity(0).withFeedForward(-power));
            elevatorMotorB.setControl(voltageOut.withSlot(1).withVelocity(0).withFeedForward(-power));
        });
    }

    public Command stopElevator() {
        return Commands.runOnce(() -> {
            elevatorMotor.stopMotor();
            elevatorMotorB.stopMotor();
        });
    }

    public void handleState(ElevatorState state) {
        this.state = state;
        if (state.isSetpoint()) {
            elevatorMotor.setControl(positionVoltage.withPosition(state.getPosition()).withSlot(0));
            elevatorMotorB.setControl(positionVoltage.withPosition(state.getPosition()).withSlot(0));
        }
    }

    public Command setState(ElevatorState state) {
        return runOnce(() -> handleState(state));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevatorAMotorPos", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevatorARPM", elevatorMotor.getRotorVelocity().getValueAsDouble());

        SmartDashboard.putNumber("elevatorBMotorPos", elevatorMotorB.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevatorBRPM", elevatorMotorB.getRotorVelocity().getValueAsDouble());

        SmartDashboard.putString("elevatorState", state.getTitle());
    }
    

}
