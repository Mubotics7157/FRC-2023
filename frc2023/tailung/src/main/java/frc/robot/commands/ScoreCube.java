package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoringPosition;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ScoreCube extends CommandBase{
      
    private SuperStructure superStructure;

    public ScoreCube(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {

        if (superStructure.getScoringPosition() == ScoringPosition.HIGH)
            superStructure.setState(SuperStructureState.CUBE_HIGH);
        else if(superStructure.getScoringPosition() == ScoringPosition.MID)
            superStructure.setState(SuperStructureState.CUBE_MID);
        else
            superStructure.setState(SuperStructureState.CUBE_HYBRID);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

    @Override
    public void end(boolean interrupted) {
    }
}