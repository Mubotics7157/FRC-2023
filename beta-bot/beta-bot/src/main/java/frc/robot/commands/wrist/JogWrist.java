package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class JogWrist extends CommandBase {

    private Wrist wrist;
    private double value;

    public JogWrist(Wrist instance, double value){

        wrist = instance;
        this.value = value;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setWristState(WristState.JOG);
    }

    @Override
    public void execute() {
        wrist.jog(value);
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setHolding();
        wrist.setWristState(WristState.SETPOINT);
    
    }


}