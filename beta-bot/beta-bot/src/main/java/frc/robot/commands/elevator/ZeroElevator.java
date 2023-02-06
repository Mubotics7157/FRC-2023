package frc.robot.commands.elevator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ZeroElevator extends CommandBase{
    
    private Elevator elevator;
    private boolean hasZeroed;

    public ZeroElevator(Elevator instance){
        elevator = instance;
        hasZeroed = false;
        addRequirements(elevator);
    }

 
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.setState(ElevatorState.ZERO);
        hasZeroed = elevator.zeroRoutine();
    }

    @Override
    public boolean isFinished() {
        return hasZeroed;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setState(ElevatorState.SETPOINT);
        
    }
}
