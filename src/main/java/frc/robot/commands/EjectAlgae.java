package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BargeController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorSim;

public class EjectAlgae extends Command {

    EndEffectorSim effectorSim;
    CommandSwerveDrivetrain drivetrain;
    EndEffector effector;
    BargeController bargeController;

    boolean finished = false;

    List<Pose2d> processorPositions = new ArrayList<>();
    
    public EjectAlgae(EndEffectorSim effectorSim, CommandSwerveDrivetrain drivetrain, EndEffector effector, BargeController bargeController) {
        this.effectorSim = effectorSim;
        this.drivetrain = drivetrain;
        this.effector = effector;
        this.bargeController = bargeController;

        processorPositions.add(new Pose2d(5.982, 0.531, Rotation2d.kZero));
        processorPositions.add(new Pose2d(11.52, 7.507, Rotation2d.kZero));
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Pose2d processorPose = drivetrain.getState().Pose.nearest(processorPositions);
        if (Math.hypot(processorPose.getX() - drivetrain.getState().Pose.getX(), processorPose.getY() - drivetrain.getState().Pose.getY()) < 1.0) {
            if (processorPose.getX() == 5.982) {
                effector.simulatedBlueScore += 6;
                effector.blueProcessorAlgae += 1;
                effectorSim.hasAlgae = false;
                bargeController.shootRedAlgae();
            } else if (processorPose.getX() == 11.52) {
                effector.simulatedRedScore += 6;
                effector.redProcessorAlgae += 1;
                effectorSim.hasAlgae = false;
                bargeController.shootBlueAlgae();
            }
        } else {
            effectorSim.hasAlgae = false;
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
