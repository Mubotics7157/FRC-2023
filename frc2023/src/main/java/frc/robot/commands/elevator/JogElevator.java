package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class JogElevator extends CommandBase {

    Elevator elevator;

    double val;

    public JogElevator(double val,Elevator instance){
        this.val = val;

        elevator = instance;
    }
    
    @Override
    public void execute() {
        elevator.setOutput(val);
        SmartDashboard.putString("run elevator","yes");
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setOutput(0);
        SmartDashboard.putString("run elevator","no");
    }
}
