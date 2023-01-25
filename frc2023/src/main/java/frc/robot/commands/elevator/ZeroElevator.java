package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends CommandBase {

    private Elevator elevator;
    
    public ZeroElevator(Elevator instance){
        elevator = instance;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPercentOutput(ElevatorConstants.ZEROING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return elevator.atZero();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPercentOutput(0);
        elevator.zeroElevator();
    }

}
