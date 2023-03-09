package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoringPosition;

public class SetScorePosition extends InstantCommand {

    private ScoringPosition position;

    public SetScorePosition(ScoringPosition position){
        this.position = position;
    }

    @Override
    public void initialize() {
        SuperStructure.getInstance().setScorePosition(position);       
    }
    
}
