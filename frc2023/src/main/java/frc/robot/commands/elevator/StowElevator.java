package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class StowElevator extends CommandBase{
    private Elevator elevator;

    public StowElevator(Elevator instance){
        elevator = instance;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorState(ElevatorState.STOW);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
    
}
