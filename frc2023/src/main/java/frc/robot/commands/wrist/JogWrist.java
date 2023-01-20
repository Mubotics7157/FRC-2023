package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class JogWrist extends CommandBase {
    
    private boolean output;
    private Wrist wrist;

    public JogWrist(boolean output, Wrist instance){
        this.output = output;

        wrist = instance;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        if(output){
            wrist.setPercentOutput(.75);

        }
        else
            wrist.setPercentOutput(-.75);
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setPercentOutput(0);
    }


}
