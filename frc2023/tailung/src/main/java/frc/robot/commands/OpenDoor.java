package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class OpenDoor extends InstantCommand {
    private SuperStructure superStructure;
    Timer timer = new Timer();

    public OpenDoor(SuperStructure instance){
        superStructure = instance;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.OPEN_DOOR);
        
    }


}
