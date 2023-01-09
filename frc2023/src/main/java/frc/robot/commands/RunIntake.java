package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private boolean reverse;

    public RunIntake(boolean reverse, Intake instance){
        this.reverse = reverse;

        intake = instance;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (reverse){
            SmartDashboard.putString("running?", "reverse");
            intake.setMotors(-SmartDashboard.getNumber("Intake Speed", 1));
        }
        else{
            intake.setMotors(SmartDashboard.getNumber("Intake Speed", 1));
            SmartDashboard.putString("running?", "forward");
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotors(0);
        SmartDashboard.putString("running?", "no");
    }

}
