package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class JogWrist extends CommandBase {
    
    private double output;
    private Wrist wrist;

    public JogWrist(double output, Wrist instance){
        this.output = output;

        wrist = instance;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.jog(output);
    }

    @Override
    public void end(boolean interrupted) {
        wrist.jog(0);
    }


}
