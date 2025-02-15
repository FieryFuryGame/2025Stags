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

    public Command move(double power) {
        return runOnce(() -> deepCageMotor.setVoltage(power));
    }

    public Command moveWithLimit(double power) {
        return runOnce(() -> deepCageMotor.setVoltage(power)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() <= 0.0 && deepCageMotor.getPosition().getValueAsDouble() >= 0.0)
        .andThen(() -> deepCageMotor.setVoltage(0.0)).onlyIf(() -> deepCageMotor.getPosition().getValueAsDouble() <= 0.0 || deepCageMotor.getPosition().getValueAsDouble() >= 0.0);
    }

    public Command activateMotionMagic() {
        return runOnce(() -> deepCageMotor.setControl(positionVoltage.withPosition(deepCageMotor.getPosition().getValueAsDouble())));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climbMotorPos", deepCageMotor.getPosition().getValueAsDouble());
    }

}
