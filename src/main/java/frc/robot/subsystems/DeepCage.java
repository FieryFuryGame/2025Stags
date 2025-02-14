package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    public Command moveWithLimit() {

    }

    public Command activateMotionMagic() {
        return runOnce(() -> deepCageMotor.setControl(positionVoltage.withPosition(deepCageMotor.getPosition().getValueAsDouble())));
    }

}
