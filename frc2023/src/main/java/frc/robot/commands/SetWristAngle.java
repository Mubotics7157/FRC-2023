package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;

    public SetWristAngle(Rotation2d setpoint,Wrist instance){
        this.setpoint = setpoint;

        wrist = instance;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setGains();
    }

    @Override
    public void execute() {
        if(Elevator.getInstance().getHeight()>9500)
            wrist.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}