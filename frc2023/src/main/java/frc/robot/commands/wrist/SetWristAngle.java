package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;
    private boolean useSD;
  

    public SetWristAngle(Rotation2d setpoint,Wrist instance, boolean useSD){
        wrist = instance;
        this.setpoint = setpoint;

        this.useSD = useSD;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setWristState(WristState.SETPOINT);
        wrist.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        if(useSD)
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Wrist setpoint", 0));

        wrist.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }

}
