package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSimExtra;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSimExtra;
import frc.robot.subsystems.ExtraDriver;

public class SimulateAlgaeIntakeExtra extends Command {

    EndEffector effector;
    EndEffectorSimExtra effectorSim;
    ExtraDriver drivetrain;
    ElevatorSimExtra elevatorSim;

    boolean finished = false;
    
    public SimulateAlgaeIntakeExtra(EndEffector effector, EndEffectorSimExtra effectorSim, ElevatorSimExtra elevatorSim, ExtraDriver drivetrain) {
        this.effector = effector;
        this.effectorSim = effectorSim;
        this.drivetrain = drivetrain;
        this.elevatorSim = elevatorSim;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(!effectorSim.simulatedBeamBreak && !effectorSim.hasAlgae) {
            Pose3d nearestAlgaePose = getClosestAlgae();
            if (Math.hypot(nearestAlgaePose.getX() - drivetrain.getState().Pose.getX(), nearestAlgaePose.getY() - drivetrain.getState().Pose.getY()) < 1) {
                if (nearestAlgaePose.getZ() == 0.9) {
                    if (effector.algaePositions.contains(nearestAlgaePose)) {
                        if (elevatorSim.percentageUp > 35 && elevatorSim.percentageUp < 45) {
                            effector.algaePositions.remove(nearestAlgaePose);
                            effectorSim.hasAlgae = true;
                        }
                    }
                } else if (nearestAlgaePose.getZ() == 1.3) {
                    if (effector.algaePositions.contains(nearestAlgaePose)) {
                        if (elevatorSim.percentageUp > 59 && elevatorSim.percentageUp < 69) {
                            effector.algaePositions.remove(nearestAlgaePose);
                            effectorSim.hasAlgae = true;
                        }
                    }
                }
            }
        }
        effector.updateAlgaeArray();
        finished = true;
    }

    public Pose3d getClosestAlgae() {
        Pose3d closest = new Pose3d(0, 0, -5, Rotation3d.kZero);
        double distance = 100.0;
        if (!effector.algaePositions.isEmpty()) {
            for (int i = 0; i < effector.algaePositions.size(); i++) {
                if (Math.hypot(effector.algaePositions.get(i).getX() - drivetrain.getState().Pose.getX(), effector.algaePositions.get(i).getY() - drivetrain.getState().Pose.getY()) < distance) {
                    closest = effector.algaePositions.get(i);
                    distance = Math.hypot(effector.algaePositions.get(i).getX() - drivetrain.getState().Pose.getX(), effector.algaePositions.get(i).getY() - drivetrain.getState().Pose.getY());
                }
            }
        }
            
        return closest;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
