package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntakeAngle extends CommandBase{
    Intake intake;
    double value;

    public MoveIntakeAngle(Intake instance, double value){
        intake = instance;

        this.value = value;
        addRequirements(intake);

    }


    @Override
    public void execute() {
        //intake.setAngle(value);
    }

    @Override
    public void end(boolean interrupted) {
        //intake.setAngle(0);
    }
}
