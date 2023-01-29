package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class JogElevator extends CommandBase {
    private Elevator elevator;
    private double val;

    public JogElevator(double val,Elevator instance){
        this.val = val;

        elevator = instance;

    }

    @Override
    public void initialize() {
        elevator.setState(ElevatorState.JOG);
    }
    @Override
    public void execute() {
        elevator.setJogInput(val);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setState(elevator.getElevatorHeight());
    }
}
