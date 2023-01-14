package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;

    public SetWristAngle(Rotation2d setpoint,Wrist instance, boolean useSD){
        this.setpoint = setpoint;

        wrist = instance;
        addRequirements(wrist);

        if(useSD)
            this.setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist setpoint", 0));


    }

    @Override
    public void initialize() {
        //setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist setpoint",0));
        wrist.setGains();
    }

    @Override
    public void execute() {
        //if(Elevator.getInstance().getHeight()>0)
            wrist.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}