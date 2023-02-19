package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class Outtake extends CommandBase {
    private IntakeState state;
    
    public Outtake(IntakeState state){
        this.state = state;
    }

    @Override
    public void execute() {
        Intake.getInstance().setIntakeState(state);
        //Intake.getInstance().setIntakeState(IntakeState.OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().setIntakeState(IntakeState.OFF);
    }
}
