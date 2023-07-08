package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class SetClimbMode extends InstantCommand {

    private SuperStructure superStructure;

    public SetClimbMode(SuperStructure superStructure){
        this.superStructure = superStructure;
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CLIMB);
    }
    
}
