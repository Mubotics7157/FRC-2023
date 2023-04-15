package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ForksConstants;
import frc.robot.subsystems.Fork;

public class QuickDeployForks extends CommandBase {

    private Fork fork;
    private boolean atSetpoint;

    public QuickDeployForks(Fork instance){
        fork = instance;
        addRequirements(fork);
        
    }

    @Override
    public void execute() {
        atSetpoint = fork.setSetpoint(ForksConstants.FORK_DEPLOY_SETPOINT);
    }

    @Override
    public boolean isFinished() {
        return atSetpoint || fork.getInitialDeployState();
    }

    @Override
    public void end(boolean interrupted) {
        fork.setInitialDeployState(true);
    }
}   