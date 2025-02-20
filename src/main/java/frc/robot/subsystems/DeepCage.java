package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeepCageConstants;

public class DeepCage extends SubsystemBase {

    TalonFX deepCageMotor = new TalonFX(DeepCageConstants.deepCageMotorID);

    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);
    
    public DeepCage() {

    }

    public Command move(double power) { // Has no limit, which means it is just for testing the deep cage, and finding positions to lock to.
        return runOnce(() -> deepCageMotor.setVoltage(power));
    }

    public Command moveWithLimit(double power) { // Will likely get stuck if outside of the range for even a second.
        return runOnce(() -> deepCageMotor.setVoltage(power)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() <= 0.0 && deepCageMotor.getPosition().getValueAsDouble() >= 0.0)
        .andThen(() -> deepCageMotor.setVoltage(0.0)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() <= 0.0 || deepCageMotor.getPosition().getValueAsDouble() >= 0.0);
    }

    public Command moveWithLimitUp(double power) { // Using this won't let the deep cage reach an upper limit.
        return runOnce(() -> deepCageMotor.setVoltage(power)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() <= 0.0)
        .andThen(() -> deepCageMotor.setVoltage(0.0)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() > 0.0);
    }

    public Command moveWithLimitDown(double power) { // Using this won't let the deep cage reach a lower limit.
        return runOnce(() -> deepCageMotor.setVoltage(power)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() >= 0.0)
        .andThen(() -> deepCageMotor.setVoltage(0.0)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() < 0.0);
    }

    public Command activateMotionMagic() { // I don't believe this is supposed to be used at any point.
        return runOnce(() -> deepCageMotor.setControl(positionVoltage.withPosition(deepCageMotor.getPosition().getValueAsDouble())));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climbMotorPos", deepCageMotor.getPosition().getValueAsDouble());
    }

}
