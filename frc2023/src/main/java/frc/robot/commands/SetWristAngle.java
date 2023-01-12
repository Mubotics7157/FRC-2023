package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private double setpoint;

    public SetWristAngle(double setpoint,Wrist instance){
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
        wrist.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}