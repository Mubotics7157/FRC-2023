package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Zero extends CommandBase {

    @Override
    public void initialize() {
        SuperStructure.getInstance().setState(SuperStructureState.ZERO);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
