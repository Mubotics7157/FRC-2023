package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Score extends CommandBase{
      
    private SuperStructure superStructure;

    public Score(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        Drive.getInstance().changeVerySlow();

        if(Intake.getInstance().isClosed()){ //if object is cone
            
            if(superStructure.getScoringHeight()) //if scoring is Mid
                superStructure.setState(SuperStructureState.CONE_HIGH);

            else if(!superStructure.getScoringHeight()) //if scoring is High
                superStructure.setState(SuperStructureState.CONE_MID);
        }

        else if(!Intake.getInstance().isClosed()){ //if object is cube

            if(superStructure.getScoringHeight()) //if scoring is Mid
                superStructure.setState(SuperStructureState.CUBE_HIGH);

            else if(!superStructure.getScoringHeight()) //if scoring is High
                superStructure.setState(SuperStructureState.CUBE_MID);
        }
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

    @Override
    public void end(boolean interrupted) {
        Drive.getInstance().changeMax();
    }

}
