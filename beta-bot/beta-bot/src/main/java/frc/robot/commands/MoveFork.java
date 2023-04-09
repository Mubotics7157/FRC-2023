package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Fork;

public class MoveFork extends CommandBase{
    private Fork fork;
    private DoubleSupplier sup;
    private boolean reverse;

    public MoveFork(Fork fork, DoubleSupplier sup, boolean reverse){
        this.fork = fork;
        this.sup = sup;
        this.reverse = reverse;

        addRequirements(fork);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(reverse)
            fork.set(-sup.getAsDouble());
        else
            fork.set(sup.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        fork.set(0);
    }
}
