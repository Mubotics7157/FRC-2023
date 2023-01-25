package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class JogElevator extends CommandBase {
    private Elevator elevator;
    private double val;

    public JogElevator(double val,Elevator instance){
        this.val = val;

        elevator = instance;

    }

    @Override
    public void execute() {
        elevator.setPercentOutput(val);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setState(elevator.getElevatorHeight());
    }
}
