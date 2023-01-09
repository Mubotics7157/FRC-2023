package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class IdleElevator extends CommandBase {

    Elevator elevator;

    double val;

    public IdleElevator(Elevator instance){
        this.val = val;

        elevator = instance;
    }
    
    @Override
    public void execute() {
        elevator.idle();
        SmartDashboard.putString("run elevator","idle");
    }

    @Override
    public void end(boolean interrupted) {
    }
}
