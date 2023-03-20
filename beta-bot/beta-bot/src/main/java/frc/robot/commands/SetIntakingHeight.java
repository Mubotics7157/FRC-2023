package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class SetIntakingHeight extends InstantCommand {
    private SuperStructure superStructure;
    private SuperStructureState state;

    public SetIntakingHeight(SuperStructure instance,SuperStructureState state){
        superStructure = instance;
        this.state = state;

        addRequirements(superStructure);
    }


        @Override
        public void execute() {
            superStructure.setState(state);
        }


}
