package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    
    Intake intake;

    public RunIntake(Intake instance){
        intake = instance;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotors(SmartDashboard.getNumber("Intake Speed", 1));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotors(0);
    }

}
