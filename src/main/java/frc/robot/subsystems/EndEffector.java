package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    TalonFX endEffectorMotor = new TalonFX(Constants.EndEffectorConstants.EndEffectorMotorID);
    
    public EndEffector() {
        // This is very useful. Does a lot.
    }

    public Command setVoltage(double power) {
        return runOnce(() -> endEffectorMotor.setVoltage(power));
    }
}