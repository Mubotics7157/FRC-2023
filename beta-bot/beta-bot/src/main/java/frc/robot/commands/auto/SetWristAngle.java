package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristAngle extends CommandBase{

    private Rotation2d setpoint;
    private boolean useSD;
    private boolean mid;

    public SetWristAngle(Rotation2d setpoint,boolean useSD, boolean mid){
        this.setpoint = setpoint;

        this.useSD = useSD;
        this.mid = mid;
    }

    @Override
    public void initialize() {
        // TODO: change the first and last line of the init to be one function so when setting a new setpoint then the mode also changes to setpoint mode automatically
        Wrist.getInstance().setWristState(WristState.SETPOINT);

        if(useSD && !mid)
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Wrist setpoint", 0));
        else if(useSD && mid){
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("mid score", 0));
        }
        
        Wrist.getInstance().setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        if(useSD && !mid)
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Wrist setpoint", 0));
        else if(useSD && mid){
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("mid score", 0));
        }

        Wrist.getInstance().setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Wrist.getInstance().atSetpoint();
    }

}
