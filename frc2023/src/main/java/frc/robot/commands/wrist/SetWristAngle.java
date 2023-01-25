package frc.robot.commands.wrist;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetWristAngle extends CommandBase{

    private Wrist wrist;
    private Rotation2d setpoint;
    private boolean useSD;

    public SetWristAngle(Rotation2d setpoint,Wrist instance, boolean useSD){
        this.setpoint = setpoint;
        this.useSD = useSD;

        wrist = instance;
        addRequirements(wrist);

       


    }

    @Override
    public void initialize() {
        //setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist setpoint",0));
        wrist.setGains();

         if(useSD)
            this.setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist setpoint", 175));
    }

    @Override
    public void execute() {

        //if(useSD)
            //this.setpoint = Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist setpoint", 175));

        //if(Elevator.getInstance().getHeight()>0)
            wrist.setSetpoint(setpoint);

            
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}