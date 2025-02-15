package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

    TalonFX endEffectorMotor = new TalonFX(Constants.EndEffectorConstants.EndEffectorMotorID);
    DigitalInput beamBreak = new DigitalInput(0);
    
    public EndEffector() {
        // This is very useful. Does a lot.
    }

    public Command setVoltage(double power) {
        return runOnce(() -> endEffectorMotor.setVoltage(power));
    }

    public boolean isCoralLoaded() {
        return !beamBreak.get();
    }

    public Command runEffector() {
        if (isCoralLoaded()) {
            return setVoltage(0.0).until(() -> !isCoralLoaded()).andThen(new WaitCommand(0.0)).andThen(setVoltage(0.0));
        } else {
            return setVoltage(0.0).until(() -> isCoralLoaded()).andThen(new WaitCommand(0.0)).andThen(setVoltage(0.0));
        }
    }
    
}