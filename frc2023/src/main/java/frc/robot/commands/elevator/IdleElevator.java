package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class IdleElevator extends CommandBase{

    private Elevator elevator;


    public IdleElevator(Elevator instance){
        elevator = instance;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.holdAtWantedState();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPercentOutput(0);       
    }
}
