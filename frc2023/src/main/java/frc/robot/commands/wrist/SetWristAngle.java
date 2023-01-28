package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;
  

    public SetWristAngle(Wrist instance, Rotation2d setpoint){
        wrist = instance;
        this.setpoint = setpoint;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        
        wrist.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        
        wrist.setWristState(WristState.SETPOINT);
    }

    @Override
    public void end(boolean interrupted) {
            wrist.setWristState(WristState.HOLD);
    }
}
