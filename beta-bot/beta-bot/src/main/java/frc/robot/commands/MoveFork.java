package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Fork;

public class MoveFork extends CommandBase{
    Fork fork;
    DoubleSupplier sup;

    public MoveFork(Fork fork, DoubleSupplier sup){
        this.fork = fork;
        this.sup = sup;

        addRequirements(fork);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        fork.set(sup.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        fork.set(0);
    }
}
