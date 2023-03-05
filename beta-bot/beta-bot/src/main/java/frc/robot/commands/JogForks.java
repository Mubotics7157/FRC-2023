package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Forks;

public class JogForks extends CommandBase {

    private Forks forks;
    public JogForks(Forks forks){

        this.forks = forks;
        addRequirements(forks);
    }
    
    @Override
    public void execute() {
        forks.setOutput(-RobotContainer.m_operatorController.getRawAxis(1)/2);
    }

    @Override
    public void end(boolean interrupted) {
        forks.setOutput(0);
    }
}
