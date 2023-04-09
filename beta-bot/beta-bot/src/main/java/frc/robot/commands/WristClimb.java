package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.Wrist.WristState;

public class WristClimb extends CommandBase {

    @Override
    public void initialize() {
        SuperStructure.getInstance().setState(SuperStructureState.CLIMB);
    }
    
    @Override
    public boolean isFinished() {
        return SuperStructure.getInstance().atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        //SuperStructure.getInstance().setState(SuperStructureState.STOWED);
    }

}
