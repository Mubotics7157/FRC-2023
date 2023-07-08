package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Zero extends CommandBase {

    @Override
    public void initialize() {
        SuperStructure.getInstance().setState(SuperStructureState.ZERO);
    }
    
    @Override
    public boolean isFinished() {
        return  SuperStructure.getInstance().isZeroed();
    }
    @Override
    public void end(boolean interrupted) {
        SuperStructure.getInstance().setState(SuperStructureState.STOWED);
    }

}
