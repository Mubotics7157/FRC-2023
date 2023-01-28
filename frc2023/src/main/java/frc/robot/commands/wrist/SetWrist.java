package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class SetWrist extends CommandBase{

    private WristState state;
    private Wrist wrist;
    private double setpoint;

    public SetWrist(WristState state, Wrist instance, double setpoint){
        wrist = instance;
        this.state = state;
        this.setpoint = setpoint;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        if(state == WristState.JOG){
            wrist.jog(setpoint);
        }
        else if(state == WristState.SETPOINT){
            wrist.setSetpoint(Rotation2d.fromDegrees(setpoint));
        }
    }

    @Override
    public void execute() {
        
        wrist.setWristState(state);
    }

    @Override
    public void end(boolean interrupted) {
        //if(state == WristState.JOG || state == WristState.SETPOINT){
            wrist.setWristState(WristState.HOLD);
        //} 
    }
}
