package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends CommandBase {
    
    private Elevator elevator;
    private double height;

    public SetElevatorHeight(double height,Elevator instance){
        this.height = height;
        elevator = instance;

        addRequirements(elevator);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
            

    }

    @Override
    public void end(boolean interrupted) {
        // elevator.setPositionHold(false);
        
    }
}
