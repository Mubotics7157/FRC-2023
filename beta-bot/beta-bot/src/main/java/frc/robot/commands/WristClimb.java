package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class WristClimb extends CommandBase {

    @Override
    public void initialize() {
        SuperStructure.getInstance().setState(SuperStructureState.CLIMB);
    }
    
    @Override
    public boolean isFinished() {
        return SuperStructure.getInstance().atSetpoint();
    }

}
