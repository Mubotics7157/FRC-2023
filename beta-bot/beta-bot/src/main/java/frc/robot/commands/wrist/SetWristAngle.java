package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Wrist.WristState;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;
    private boolean useSD;
    private boolean mid;

    public SetWristAngle(Rotation2d setpoint,Wrist instance, boolean useSD, boolean mid){
        wrist = instance;
        this.setpoint = setpoint;

        this.useSD = useSD;
        this.mid = mid;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setWristState(WristState.SETPOINT);

        if(useSD && !mid)
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Wrist setpoint", -117));
        else if(useSD && mid){
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("mid score", -135));
        }
        
        wrist.setSetpoint(setpoint);

    }

    @Override
    public void execute() {
        if(useSD && !mid)
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("Wrist setpoint", 0));
        else if(useSD && mid){
            setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("mid score", 0));
        }

        wrist.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
       
        if(setpoint.getDegrees() == -7){
            IntakeVision.getInstance().setObjectOffset();
        }
    }

}
