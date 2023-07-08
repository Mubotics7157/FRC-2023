package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoringPosition;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ScoreCone extends CommandBase{
      
    private SuperStructure superStructure;

    public ScoreCone(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {

        if (superStructure.getScoringPosition() == ScoringPosition.HIGH)
            superStructure.setState(SuperStructureState.CONE_HIGH);
        else if(superStructure.getScoringPosition() == ScoringPosition.MID)
            superStructure.setState(SuperStructureState.CONE_MID);
        else if(superStructure.getScoringPosition() == ScoringPosition.HYBRID)
            superStructure.setState(SuperStructureState.CONE_SNIPER);
        else
            superStructure.setState(SuperStructureState.CONE_MID_SUPER);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

    @Override
    public void end(boolean interrupted) {
    }
}