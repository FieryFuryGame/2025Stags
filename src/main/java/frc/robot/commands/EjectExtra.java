package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BargeController;
import frc.robot.subsystems.ElevatorSimExtra;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSimExtra;
import frc.robot.subsystems.ExtraDriver;

public class EjectExtra extends Command {

    EndEffector effector;
    EndEffectorSimExtra effectorSim;
    ElevatorSimExtra elevatorSim;
    ExtraDriver drivetrain;
    BargeController bargeController;

    boolean finished = false;
    
    public EjectExtra(EndEffector effector, EndEffectorSimExtra effectorSim, ElevatorSimExtra elevatorSim, ExtraDriver drivetrain, BargeController bargeController) {
        this.effector = effector;
        this.effectorSim = effectorSim;
        this.elevatorSim = elevatorSim;
        this.drivetrain = drivetrain;
        this.bargeController = bargeController;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (effectorSim.simulatedBeamBreak) {
            new DecideWhereToPlaceCoralExtra(elevatorSim, drivetrain, effector, effectorSim).schedule();
        } else if (effectorSim.hasAlgae) {
            new EjectAlgaeExtra(effectorSim, drivetrain, effector, bargeController).schedule();
        }
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
