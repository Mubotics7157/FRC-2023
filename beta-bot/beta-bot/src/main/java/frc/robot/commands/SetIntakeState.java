package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntakeState extends CommandBase {
    private IntakeState state;
    
    public SetIntakeState(IntakeState state){
        this.state = state;
    }

    @Override
    public void execute() {
        Intake.getInstance().setIntakeState(state);
    }

}
