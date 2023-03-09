package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
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
        //Drive.getInstance().changeVerySlow();

        if (superStructure.getScoringPosition() == ScoringPosition.HIGH)
            superStructure.setState(SuperStructureState.CONE_HIGH);
        else if(superStructure.getScoringPosition() == ScoringPosition.MID)
            superStructure.setState(SuperStructureState.CONE_MID);
        else
            superStructure.setState(SuperStructureState.CONE_SNIPER);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

    @Override
    public void end(boolean interrupted) {
        //Drive.getInstance().changeMax();
    }
}