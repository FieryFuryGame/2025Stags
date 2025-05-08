package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSim;

public class SimulateAlgaeIntake extends Command {

    EndEffector effector;
    EndEffectorSim effectorSim;
    CommandSwerveDrivetrain drivetrain;
    
    public SimulateAlgaeIntake(EndEffector effector, EndEffectorSim effectorSim, CommandSwerveDrivetrain drivetrain) {
        this.effector = effector;
        this.effectorSim = effectorSim;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(!effector.simulatedBeamBreak && !effectorSim.hasAlgae) {

        }
    }

    public Pose3d getClosestAlgae() {
        
        return null;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
