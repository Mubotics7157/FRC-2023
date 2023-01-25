package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends CommandBase {
    
    private Elevator elevator;
    private double height;
    private boolean useSD;
    private boolean atHeight = false;

    public SetElevatorHeight(double height,Elevator instance, boolean useSD){
        this.height = height;
        this.useSD = useSD;
        elevator = instance;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {

        if(useSD)
            this.height = SmartDashboard.getNumber("elevator setpoint", 0);
    }

    @Override
    public void execute() {
        
        atHeight = elevator.setState(height);

    }

    @Override
    public boolean isFinished() {
        return atHeight;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.holdAtWantedState();
        
    }
}
