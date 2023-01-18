package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends CommandBase {
    
    private Elevator elevator;
    private double height;
    private boolean useSD;

    public SetElevatorHeight(double height,Elevator instance, boolean useSD){
        this.height = height;
        this.useSD = useSD;
        elevator = instance;

        addRequirements(elevator);

        if(useSD)
            this.height = SmartDashboard.getNumber("elevator setpoint", 17791);
    }

    @Override
    public void initialize() {
        elevator.setGains(.08);
        elevator.setPositionHold(true);

        if(useSD)
            this.height = SmartDashboard.getNumber("elevator setpoint", 17791);
        //height = SmartDashboard.getNumber("elevator setpoint", 0);
    }

    @Override
    public void execute() {
        //if(useSD)
            //this.height = SmartDashboard.getNumber("elevator setpoint", 17791);
        
            
        elevator.setSetpoint(height);
        SmartDashboard.putNumber("elevator height setpoint", height);
        //SmartDashboard.putNumber("bruh", SmartDashboard.getNumber("elevator onboard encoder", 0));

    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPositionHold(false);
        
    }
}
