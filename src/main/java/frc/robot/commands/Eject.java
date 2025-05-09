package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BargeController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSim;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSim;

public class Eject extends Command {

    EndEffector effector;
    EndEffectorSim effectorSim;
    ElevatorSim elevatorSim;
    CommandSwerveDrivetrain drivetrain;
    BargeController bargeController;

    boolean finished = false;
    
    public Eject(EndEffector effector, EndEffectorSim effectorSim, ElevatorSim elevatorSim, CommandSwerveDrivetrain drivetrain, BargeController bargeController) {
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
        if (effector.simulatedBeamBreak) {
            new DecideWhereToPlaceCoral(elevatorSim, drivetrain, effector).schedule();
        } else if (effectorSim.hasAlgae) {
            new EjectAlgae(effectorSim, drivetrain, effector, bargeController).schedule();
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
